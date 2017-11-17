/*
 * drivers/input/keyboard/pxa3xx_keypad.c
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/gpio.h>

#include <asm/arch/pxa3xx_keypad.h>

#if defined(CONFIG_PXA3xx_DVFM)
#include <linux/notifier.h>
#include <linux/timer.h>
#include <asm/arch/dvfm.h>
#include <asm/arch/pxa3xx_dvfm.h>
#include <asm/arch/pxa3xx_pm.h>
#endif
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#include <asm/arch/mfp-pxa9xx.h>



/*
 * Keypad Controller registers
 */
#define KPC             0x0000 /* Keypad Control register */
#define KPDK            0x0008 /* Keypad Direct Key register */
#define KPREC           0x0010 /* Keypad Rotary Encoder register */
#define KPMK            0x0018 /* Keypad Matrix Key register */
#define KPAS            0x0020 /* Keypad Automatic Scan register */

/* Keypad Automatic Scan Multiple Key Presser register 0-3 */
#define KPASMKP0        0x0028
#define KPASMKP1        0x0030
#define KPASMKP2        0x0038
#define KPASMKP3        0x0040
#define KPKDI           0x0048

#ifdef CONFIG_CPU_PXA930
#define ERCR_OFF	0xc
#define SBCR_OFF	0x4
#endif

/* bit definitions */
#define KPC_MKRN(n)	((((n) & 0x7) - 1) << 26) /* matrix key row number */
#define KPC_MKCN(n)	((((n) & 0x7) - 1) << 23) /* matrix key column number */
#define KPC_DKN(n)	((((n) & 0x7) - 1) << 6)  /* direct key number */

#define KPC_AS          (0x1 << 30)  /* Automatic Scan bit */
#define KPC_ASACT       (0x1 << 29)  /* Automatic Scan on Activity */
#define KPC_MI          (0x1 << 22)  /* Matrix interrupt bit */
#define KPC_IMKP        (0x1 << 21)  /* Ignore Multiple Key Press */

#define KPC_MS(n)	(0x1 << (13 + (n)))	/* Matrix scan line 'n' */
#define KPC_MS_ALL      (0xff << 13)

#define KPC_ME          (0x1 << 12)  /* Matrix Keypad Enable */
#define KPC_MIE         (0x1 << 11)  /* Matrix Interrupt Enable */
#define KPC_DK_DEB_SEL	(0x1 <<  9)  /* Direct Keypad Debounce Select */
#define KPC_DI          (0x1 <<  5)  /* Direct key interrupt bit */
#define KPC_RE_ZERO_DEB (0x1 <<  4)  /* Rotary Encoder Zero Debounce */
#define KPC_REE1        (0x1 <<  3)  /* Rotary Encoder1 Enable */
#define KPC_REE0        (0x1 <<  2)  /* Rotary Encoder0 Enable */
#define KPC_DE          (0x1 <<  1)  /* Direct Keypad Enable */
#define KPC_DIE         (0x1 <<  0)  /* Direct Keypad interrupt Enable */

#define KPDK_DKP        (0x1 << 31)
#define KPDK_DK(n)	((n) & 0xff)

#define KPREC_OF1       (0x1 << 31)
#define KPREC_UF1       (0x1 << 30)
#define KPREC_OF0       (0x1 << 15)
#define KPREC_UF0       (0x1 << 14)

#define KPREC_RECOUNT0(n)	((n) & 0xff)
#define KPREC_RECOUNT1(n)	(((n) >> 16) & 0xff)

#define KPMK_MKP        (0x1 << 31)
#define KPAS_SO         (0x1 << 31)
#define KPASMKPx_SO     (0x1 << 31)

#define KPAS_MUKP(n)	(((n) >> 26) & 0x1f)
#define KPAS_RP(n)	(((n) >> 4) & 0xf)
#define KPAS_CP(n)	((n) & 0xf)

#define KPASMKP_MKC_MASK	(0xff)

#define keypad_readl(off)	__raw_readl(keypad->mmio_base + (off))
#define keypad_writel(off, v)	__raw_writel((v), keypad->mmio_base + (off))

#ifdef CONFIG_CPU_PXA930
#define ercr_readl(off)		__raw_readl(keypad->ercr_base + (off))
#define ercr_writel(off, v)	__raw_writel((v), keypad->ercr_base + (off))
#endif

#define MAX_MATRIX_KEY_NUM	(8 * 8)

#if defined(CONFIG_PXA3xx_DVFM)
#define D2_STABLE_JIFFIES               6

static int keyevent_enable = 0;
static int keypad_notifier_freq(struct notifier_block *nb,
			unsigned long val, void *data);
static struct notifier_block notifier_freq_block = {
	.notifier_call = keypad_notifier_freq,
};

static struct dvfm_lock dvfm_lock = {
	.lock		= SPIN_LOCK_UNLOCKED,
	.dev_idx	= -1,
	.count		= 0,
};

static struct timer_list kp_timer;
#endif
//fang for autofoucs keypad
static struct timer_list cam_timer;
#define TIMEOUT_VALUE   (2*HZ)
#define SET_TIMER                     \
		do {                        \
				mod_timer(&cam_timer, jiffies + TIMEOUT_VALUE);   \
		} while (0)

#define DEL_TIMER		\
		do {			\
				del_timer(&cam_timer);	\
		} while (0)

struct pxa3xx_keypad {
	struct pxa3xx_keypad_platform_data *pdata;

	/* Memory Mapped Register */
	struct resource *mem;
	void __iomem *mmio_base;
#ifdef CONFIG_CPU_PXA930
	struct resource *ercr_mem;
	void __iomem *ercr_base;
	u32 prev_ercr;
#endif
	struct clk *clk;
	struct input_dev *input_dev;

	int irq;

	/* matrix key code map */
	unsigned int matrix_keycodes[MAX_MATRIX_KEY_NUM];

	/* state row bits of each column scan */
	uint32_t matrix_key_state[MAX_MATRIX_KEY_COLS];
	uint32_t direct_key_state;

	unsigned int direct_key_mask;

	unsigned int rotary_count;

	int rotary_rel_code[2];
	int rotary_up_key[2];
	int rotary_down_key[2];
#ifdef CONFIG_CPU_PXA930
	int enhanced_rotary_rel_code;
	int enhanced_rotary_up_key;
	int enhanced_rotary_down_key;
#endif
	unsigned int lock;
};
static struct pxa3xx_keypad *keypad;

enum keypad_lock_status {
	UNLOCKED 	= 0,
	LOCKED 		= 1,
};

#define LC6830_INVENTEC_KEYPAD

static inline void keypad_lock(struct pxa3xx_keypad *keypad,
			enum keypad_lock_status value)
{
	if (value == UNLOCKED && keypad->lock == LOCKED)
	{
		keypad->lock = value;
	}
	else if (value == LOCKED && keypad->lock == UNLOCKED)
	{
		keypad->lock = value;
	}
	else
	      printk(KERN_WARNING "%s: getting keypad lock command value = %d\n",__func__, value);
}

static ssize_t keypad_lock_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct pxa3xx_keypad *keypad = dev_get_drvdata(dev);
	if( keypad )
	      return sprintf(buf, "%d\n", keypad->lock);
	else
	      return 0;
}

static ssize_t keypad_lock_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct pxa3xx_keypad *keypad = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
	      count++;

	if (count == size) {
		ret = count;
		keypad_lock(keypad, state);
	}

	return ret;
}

static DEVICE_ATTR(lock, S_IRUGO | S_IWUSR, keypad_lock_show, keypad_lock_store); 


static void pxa3xx_keypad_build_keycode(struct pxa3xx_keypad *keypad)
{
	struct pxa3xx_keypad_platform_data *pdata = keypad->pdata;
	struct input_dev *input_dev = keypad->input_dev;
	unsigned int *key;
	int i;

	key = &pdata->matrix_key_map[0];
	for (i = 0; i < pdata->matrix_key_map_size; i++, key++) {
		int row = ((*key) >> 28) & 0xf;
		int col = ((*key) >> 24) & 0xf;
		int code = (*key) & 0xffffff;

		keypad->matrix_keycodes[(row << 3) + col] = code;
		set_bit(code, input_dev->keybit);
	}

#ifdef CONFIG_TOUCH_KEYPAD
	set_bit(KEY_SEND,input_dev->keybit);
	set_bit(KEY_MENU,input_dev->keybit);
	set_bit(KEY_HOME,input_dev->keybit);
	set_bit(KEY_BACK,input_dev->keybit);
	set_bit(KEY_END,input_dev->keybit);
#endif
	set_bit(KEY_MOBILETV,input_dev->keybit);
	set_bit(KEY_CAMERA,input_dev->keybit);
	set_bit(KEY_FOCUS,input_dev->keybit);

	if(pdata->direct_key_num) {
		for (i = 0; i < pdata->direct_key_num; i++) {
			set_bit(pdata->direct_key_map[i], input_dev->keybit);
		}
	}
	keypad->rotary_up_key[0] = pdata->rotary0_up_key;
	keypad->rotary_up_key[1] = pdata->rotary1_up_key;
	keypad->rotary_down_key[0] = pdata->rotary0_down_key;
	keypad->rotary_down_key[1] = pdata->rotary1_down_key;
	keypad->rotary_rel_code[0] = pdata->rotary0_rel_code;
	keypad->rotary_rel_code[1] = pdata->rotary1_rel_code;
#ifdef CONFIG_CPU_PXA930
	keypad->enhanced_rotary_up_key = pdata->enhanced_rotary_up_key;
	keypad->enhanced_rotary_down_key = pdata->enhanced_rotary_down_key;
	keypad->enhanced_rotary_rel_code = pdata->enhanced_rotary_rel_code;
#endif

	if (pdata->rotary0_up_key && pdata->rotary0_down_key) {
		set_bit(pdata->rotary0_up_key, input_dev->keybit);
		set_bit(pdata->rotary0_down_key, input_dev->keybit);
	} else
	      set_bit(pdata->rotary0_rel_code, input_dev->relbit);

	if (pdata->rotary1_up_key && pdata->rotary1_down_key) {
		set_bit(pdata->rotary1_up_key, input_dev->keybit);
		set_bit(pdata->rotary1_down_key, input_dev->keybit);
	} else
	      set_bit(pdata->rotary1_rel_code, input_dev->relbit);

#ifdef CONFIG_CPU_PXA930
	if (pdata->enhanced_rotary_up_key && pdata->enhanced_rotary_down_key) {
		set_bit(pdata->enhanced_rotary_up_key, input_dev->keybit);
		set_bit(pdata->enhanced_rotary_down_key, input_dev->keybit);
	} else
	      set_bit(pdata->enhanced_rotary_rel_code, input_dev->relbit);
#endif
}

#ifdef CONFIG_DEBUG_TRIGGER_PANIC
static inline unsigned int get_time_ms(void)
{
	return OSCR / 3250;
}
#endif

static inline int input_trigger_event(
			struct pxa3xx_keypad_platform_data *pdata, int key, int status)
{
#ifdef CONFIG_DEBUG_TRIGGER_PANIC
	static int key0, st0, key1, st1, key2, st2, trigger_key;
	static unsigned long trigger_time;

	/* 
	 * holding trigger keys for 10 seconds will trigger the panic log,
	 * for 5 seconds will dump system status but will not trigger panic.
	 */
	if (!pdata->trigger_key0 && !pdata->trigger_key1 && !pdata->trigger_key2)
	      return 0;

	if (pdata->trigger_key0 && (key == pdata->trigger_key0)) {
		key0 = key;
		st0 = status;
	}

	if (pdata->trigger_key1 && (key == pdata->trigger_key1)) {
		key1 = key;
		st1 = status;
	}

	if (pdata->trigger_key2 && (key == pdata->trigger_key2)) {
		key2 = key;
		st2 = status;
	}

	if ((!pdata->trigger_key0 || (pdata->trigger_key0 && (key0 == pdata->trigger_key0) && st0)) && 
				(!pdata->trigger_key1 || (pdata->trigger_key1 && (key1 == pdata->trigger_key1) && st1)) && 
				(!pdata->trigger_key2 || (pdata->trigger_key2 && (key2 == pdata->trigger_key2) && st2))) {
		trigger_key = 1;
		trigger_time = get_time_ms();
		printk(KERN_EMERG "Panic key is pressed, Please hold on 10 seconds to trigger a panic.\n");
		printk(KERN_EMERG "OSCR4: 0x%x  OSMR4: 0x%x OIER: 0x%x OSSR: 0x%x\n", OSCR4, OSMR4, OIER, OSSR);
	} else {
		if (trigger_key == 1) {
			trigger_time = get_time_ms() - trigger_time;
			if (trigger_time >= 10000) {
				show_state();
				panic("Panic key triggers panic!");
				printk(KERN_EMERG "OSCR4: 0x%x  OSMR4: 0x%x OIER: 0x%x OSSR: 0x%x\n", OSCR4, OSMR4, OIER, OSSR);
			} else if (trigger_time >= 5000) {
				show_state();
				dump_stack();
			}
			trigger_key = 0;
		} 
	}
#endif
	return 0;
}

static void pxa3xx_input_report_key(struct pxa3xx_keypad *keypad, unsigned int code, int value)
{
	if ((keypad->lock == LOCKED) && (code != KEY_POWER) && (code != KEY_HOOKSWITCH) )
	      pr_info("%s: keypad is locked\n", __func__);
	else 
	      input_report_key(keypad->input_dev, code, value);

	input_trigger_event(keypad->pdata, code, value);
}

static inline unsigned int lookup_matrix_keycode(
			struct pxa3xx_keypad *keypad, int row, int col)
{
	return keypad->matrix_keycodes[(row << 3) + col];
}

static void pxa3xx_keypad_scan_matrix(struct pxa3xx_keypad *keypad)
{
	struct pxa3xx_keypad_platform_data *pdata = keypad->pdata;
	int row, col, num_keys_pressed = 0;
	uint32_t new_state[MAX_MATRIX_KEY_COLS];
	uint32_t kpas = keypad_readl(KPAS);

	num_keys_pressed = KPAS_MUKP(kpas);

	memset(new_state, 0, sizeof(new_state));

	if (num_keys_pressed == 0)
	      goto scan;

	if (num_keys_pressed == 1) {
		col = KPAS_CP(kpas);
		row = KPAS_RP(kpas);

		/* if invalid row/col, treat as no key pressed */
		if (col >= pdata->matrix_key_cols ||
					row >= pdata->matrix_key_rows)
		      goto scan;

		new_state[col] = (1 << row);
		goto scan;
	}

	if (num_keys_pressed > 1) {
		uint32_t kpasmkp0 = keypad_readl(KPASMKP0);
		uint32_t kpasmkp1 = keypad_readl(KPASMKP1);
		uint32_t kpasmkp2 = keypad_readl(KPASMKP2);
		uint32_t kpasmkp3 = keypad_readl(KPASMKP3);

		new_state[0] = kpasmkp0 & KPASMKP_MKC_MASK;
		new_state[1] = (kpasmkp0 >> 16) & KPASMKP_MKC_MASK;
		new_state[2] = kpasmkp1 & KPASMKP_MKC_MASK;
		new_state[3] = (kpasmkp1 >> 16) & KPASMKP_MKC_MASK;
		new_state[4] = kpasmkp2 & KPASMKP_MKC_MASK;
		new_state[5] = (kpasmkp2 >> 16) & KPASMKP_MKC_MASK;
		new_state[6] = kpasmkp3 & KPASMKP_MKC_MASK;
		new_state[7] = (kpasmkp3 >> 16) & KPASMKP_MKC_MASK;
	}
scan:
	for (col = 0; col < pdata->matrix_key_cols; col++) {
		uint32_t bits_changed;

		bits_changed = keypad->matrix_key_state[col] ^ new_state[col];
		if (bits_changed == 0)
		      continue;

		for (row = 0; row < pdata->matrix_key_rows; row++) {
			if ((bits_changed & (1 << row)) == 0)
			      continue;

			printk(KERN_WARNING "keycode %d\n", lookup_matrix_keycode(keypad, row, col));
			pxa3xx_input_report_key(keypad,
						lookup_matrix_keycode(keypad, row, col),
						new_state[col] & (1 << row));
		}
	}
	input_sync(keypad->input_dev);
	memcpy(keypad->matrix_key_state, new_state, sizeof(new_state));
}
#ifdef CONFIG_TOUCH_KEYPAD
static struct pxa3xx_keypad* touch_keypad = 0;

void input_touch_keypad(int key_value, int status)
{
		if(!touch_keypad)
		{
				printk("touch keypad is null\n");
				return;
		}
		input_report_key(touch_keypad->input_dev,key_value,status);
		input_trigger_event(touch_keypad->input_dev,key_value,status);
		input_sync(touch_keypad->input_dev);
}
#endif

#define DEFAULT_KPREC	(0x007f007f)

#if 1 
static int calc_rotary_delta(uint32_t kprec)
{
	if (kprec & KPREC_OF0)
	      return (kprec & 0xff) + 0x7f;
	else if (kprec & KPREC_UF0)
	      return (kprec & 0xff) - 0x7f - 0xff;
	else
	      return (kprec & 0xff) - 0x7f;
}
#endif

#ifdef	CONFIG_CPU_PXA930
/*Read enhanced rotary key*/
static int read_enh_rot_key(struct pxa3xx_keypad *keypad, int *key)
{
	u32 curr_ercr = ercr_readl(ERCR_OFF) & 0xf;

	/*assume that increases 10 at a time is impossible,
	  otherwise overflow/underflow happens.*/
	if (0x0A < abs(curr_ercr - keypad->prev_ercr))
	      if (curr_ercr > keypad->prev_ercr)	/*Underflow happens*/
		    *key = keypad->enhanced_rotary_up_key;
	      else					/*Overflow happens*/
		    *key = keypad->enhanced_rotary_down_key;

	else				/*Normal increament or decreament*/
	      *key = curr_ercr > keypad->prev_ercr ?
		      keypad->enhanced_rotary_down_key:
		      keypad->enhanced_rotary_up_key;

	keypad->prev_ercr = curr_ercr;

	return 0;
}

static void clear_sbcr(struct pxa3xx_keypad *keypad)
{
	ercr_writel(SBCR_OFF, ercr_readl(SBCR_OFF) | (1 << 5));
	ercr_writel(SBCR_OFF, ercr_readl(SBCR_OFF) & ~(1 << 5));
}

static irqreturn_t enhanced_rotary_interrupt(int irq, void *dev_id)
{
	struct pxa3xx_keypad *keypad = dev_id;
	int key;

	read_enh_rot_key(keypad, &key);
	clear_sbcr(keypad);

	if ((key == keypad->enhanced_rotary_up_key) ||
				(key == keypad->enhanced_rotary_down_key)) {
		pxa3xx_input_report_key(keypad, key, 1);
		pxa3xx_input_report_key(keypad, key, 0);
		return IRQ_HANDLED;
	}

	input_sync(keypad->input_dev);

	return IRQ_HANDLED;
}
#endif

#if 1 
static void report_rotary_event(struct pxa3xx_keypad *keypad, int r, int delta)
{
	int i;
	if (delta == 0)
	      return;

	if (keypad->rotary_up_key[r] && keypad->rotary_down_key[r]) {
		int keycode = (delta < 0) ?
			keypad->rotary_up_key[r] : keypad->rotary_down_key[r];

		if ((((delta > 0)?delta:(-delta)) >> 2) > 0)
		      keypad_writel(KPREC, DEFAULT_KPREC);
		for (i = 0;i < (((delta > 0)?delta:(-delta)) >> 2);i++){
			/* simulate a press-n-release */
			pxa3xx_input_report_key(keypad, keycode, 1);
			pxa3xx_input_report_key(keypad, keycode, 0);
		}
	} else
	      input_report_rel(keypad->input_dev,
				      keypad->rotary_rel_code[r], delta);

	//	input_sync(keypad->input_dev);
}

static void pxa3xx_keypad_scan_rotary(struct pxa3xx_keypad *keypad)
{
	struct pxa3xx_keypad_platform_data *pdata = keypad->pdata;
	uint32_t kprec;

	/* read and reset to default count value */
	kprec = keypad_readl(KPREC);
	//	keypad_writel(KPREC, DEFAULT_KPREC);

	if (pdata->enable_rotary0)
	      report_rotary_event(keypad, 0, calc_rotary_delta(kprec));

	if (pdata->enable_rotary1)
	      report_rotary_event(keypad, 1, calc_rotary_delta(kprec));
}
#endif

static void pxa3xx_keypad_scan_direct(struct pxa3xx_keypad *keypad)
{
	struct pxa3xx_keypad_platform_data *pdata = keypad->pdata;
	unsigned int new_state;
	uint32_t kpdk, bits_changed;
	int i;

	kpdk = keypad_readl(KPDK);

	printk("###direct key interrupt###\n");//aaa
	if (pdata->enable_rotary0 || pdata->enable_rotary1)
	      pxa3xx_keypad_scan_rotary(keypad);

	if (pdata->direct_key_map == NULL)
	{
		  printk("###direct_key_map NULL###\n");
	      return;
	}
	new_state = KPDK_DK(kpdk) & keypad->direct_key_mask;
	bits_changed = keypad->direct_key_state ^ new_state;

	if (bits_changed == 0)
	      return;

	for (i = 0; i < pdata->direct_key_num; i++) {
		if (bits_changed & (1 << i)) {
			printk("###direct key: %d###\n", pdata->direct_key_map[i]);
			pxa3xx_input_report_key(keypad,
						pdata->direct_key_map[i],
						(new_state & (1 << i)));
		}
	}
	//input_sync(keypad->input_dev);
	keypad->direct_key_state = new_state;
}

static int pxa3xx_keypad_interrupt(int irq, void *dev_id)
{
	struct pxa3xx_keypad *keypad = dev_id;
	uint32_t kpc = keypad_readl(KPC);

	printk("######pxa3xx_keypad_interrupt######\n");
	if (kpc & KPC_MI)
	{
		  printk("###call pxa3xx_keypad_scan_matrix###\n");
	      pxa3xx_keypad_scan_matrix(keypad);
	}
	if (kpc & KPC_DI)
	{
		  printk("###call pxa3xx_keypad_scan_direct###\n");
	      pxa3xx_keypad_scan_direct(keypad);
	}
	return IRQ_HANDLED;
}

static void pxa3xx_keypad_config(struct pxa3xx_keypad *keypad)
{
	struct pxa3xx_keypad_platform_data *pdata = keypad->pdata;
	unsigned int mask = 0, direct_key_num = 0;
	uint32_t kpc = 0;

	/* enable matrix key with automatic scan */
	if (pdata->matrix_key_rows && pdata->matrix_key_cols) {
		kpc |=  KPC_ASACT | KPC_MIE | KPC_ME | KPC_MS_ALL;
		kpc |=  KPC_MKRN(pdata->matrix_key_rows) |
			KPC_MKCN(pdata->matrix_key_cols);
	}

	/* enable rotary key, debounce interval same as direct keys */
	if (pdata->enable_rotary0) {
		mask |= 0x03;
		direct_key_num = 2;
		kpc |= KPC_REE0;
	}

	if (pdata->enable_rotary1) {
		mask |= 0x0c;
		direct_key_num = 4;
		kpc |= KPC_REE1;
	}

	if (pdata->direct_key_num > direct_key_num)
	      direct_key_num = pdata->direct_key_num;

	keypad->direct_key_mask = ((1 << direct_key_num) - 1) & ~mask;

	/* enable direct key */
	if (direct_key_num)
	      kpc |= KPC_DE | KPC_DIE | KPC_DKN(direct_key_num);

	kpc |= KPC_DK_DEB_SEL;

	keypad->rotary_count = 0x7f;

	keypad_writel(KPC, kpc);
	keypad_writel(KPREC, DEFAULT_KPREC);
	keypad_writel(KPKDI, pdata->debounce_interval & 0xff);

#ifdef CONFIG_CPU_PXA930
	if ((cpu_is_pxa930() || cpu_is_pxa935()) &&  \
				pdata->enable_enhanced_rotary)
	      clear_sbcr(keypad);
#endif
}

static int pxa3xx_keypad_open(struct input_dev *dev)
{
	struct pxa3xx_keypad *keypad = dev->private;

	clk_enable(keypad->clk);
	pxa3xx_keypad_config(keypad);

	return 0;
}

static void pxa3xx_keypad_close(struct input_dev *dev)
{
	struct pxa3xx_keypad *keypad = dev->private;

	clk_disable(keypad->clk);
}

#ifdef CONFIG_ANDROID_POWER
static android_suspend_lock_t pxa3xx_keypad_suspend_lock;
#endif

#if defined(CONFIG_PXA3xx_DVFM)
static void set_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count++ == 0) {
		/* Disable lowpower mode */
		dvfm_disable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_disable_op_name("D2", dvfm_lock.dev_idx);
		if (cpu_is_pxa935())
		      dvfm_disable_op_name("CG", dvfm_lock.dev_idx);
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void unset_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count == 0) {
		printk(KERN_WARNING "Keypad constraint has been removed.\n");
	} else if (--dvfm_lock.count == 0) {
		/* Enable lowpower mode */
		dvfm_enable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_enable_op_name("D2", dvfm_lock.dev_idx);
		if (cpu_is_pxa935())
		      dvfm_enable_op_name("CG", dvfm_lock.dev_idx);
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

/*
 * FIXME: Here a timer is used to disable entering D1/D2 for a while.
 * Because keypad event wakeup system from D1/D2 mode. But keypad device
 * can't detect the interrupt since it's in standby state.
 * Keypad device need time to detect it again. So we use a timer here.
 * D1/D2 idle is determined by idle time. It's better to comine these
 * timers together.
 */
static void keypad_timer_handler(unsigned long data)
{
	unset_dvfm_constraint();
}

extern void get_wakeup_source(pm_wakeup_src_t *);

static int keypad_notifier_freq(struct notifier_block *nb,
			unsigned long val, void *data)
{
	struct dvfm_freqs *freqs = (struct dvfm_freqs *)data;
	struct op_info *new = NULL;
	struct dvfm_md_opt *op;
	pm_wakeup_src_t src;

	if (freqs)
	      new = &freqs->new_info;
	else
	      return 0;

	op = (struct dvfm_md_opt *)new->op;
	if (val == DVFM_FREQ_POSTCHANGE) {
		if ((op->power_mode == POWER_MODE_D1) ||
					(op->power_mode == POWER_MODE_D2) ||
					(op->power_mode == POWER_MODE_CG)) {
			get_wakeup_source(&src);
			if (src.bits.mkey || src.bits.dkey) {
				/* If keypad event happens and wake system
				 * from D1/D2. Disable D1/D2 to make keypad
				 * work for a while.
				 */
				kp_timer.expires = jiffies + D2_STABLE_JIFFIES;
				add_timer(&kp_timer);
				set_dvfm_constraint();
#ifdef CONFIG_ANDROID_POWER
				android_lock_suspend_auto_expire(&pxa3xx_keypad_suspend_lock, D2_STABLE_JIFFIES);
#endif
			}
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_PM 
static int pxa3xx_keypad_suspend(struct platform_device *dev, pm_message_t state)
{
	struct pxa3xx_keypad *keypad = platform_get_drvdata(dev);

	clk_disable(keypad->clk);
	return 0;
}

static int pxa3xx_keypad_resume(struct platform_device *dev)
{
	struct pxa3xx_keypad *keypad = platform_get_drvdata(dev);
	struct input_dev *input_dev = keypad->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users) {
		clk_enable(keypad->clk);
		pxa3xx_keypad_config(keypad);
	}

	mutex_unlock(&input_dev->mutex);

	return 0;
}
#else
#define pxa3xx_keypad_suspend	NULL
#define pxa3xx_keypad_resume	NULL
#endif

#if defined(CONFIG_KEYBOARD_PXA3xx_LOCK) || defined(CONFIG_ANDROID_POWER)
void keypad_android_suspend_handler(android_early_suspend_t *h)
{
	printk(KERN_DEBUG "keypad suspend...\n");
	keypad->lock = LOCKED;
	return;
}

void keypad_android_resume_handler(android_early_suspend_t *h)
{
	printk(KERN_DEBUG "keypad resume...\n");
	keypad->lock = UNLOCKED;
	return;
}

static android_late_suspend_t keypad_android_suspend = {
	.level = 5,
	.suspend = keypad_android_suspend_handler,
	.resume = keypad_android_resume_handler,
};
#endif

#ifdef LC6830_INVENTEC_KEYPAD

#define CAMERA_KEY_GPIO		mfp_to_gpio(MFP_PIN_GPIO21)
#define MOBILETV_KEY_GPIO		mfp_to_gpio(MFP_PIN_GPIO42)

static int af_flag = 0;

static void cam_timer_handler(unsigned long data)
{
	af_flag = 1;
	pxa3xx_input_report_key(keypad, KEY_CAMERA, 0);
	pxa3xx_input_report_key(keypad, KEY_FOCUS, data);
}

static int cam_key_detect_handle(int irq, void *dev_id)
{
	int cam_key_status; //0:pressed, 1:unpressed
	
	cam_key_status = gpio_get_value(CAMERA_KEY_GPIO);
	if(cam_key_status == 0)
		cam_key_status = 2097152;
	else
		cam_key_status = 0;
	
	if(cam_key_status != 0)
	{
		cam_timer.data = cam_key_status;
		SET_TIMER;
		pxa3xx_input_report_key(keypad, KEY_CAMERA, cam_key_status);
	}
	else
	{
		cam_timer.data = 0;
		DEL_TIMER;
		if(af_flag == 1)
		{
			pxa3xx_input_report_key(keypad, KEY_CAMERA, 2097152);
			pxa3xx_input_report_key(keypad, KEY_CAMERA, cam_key_status);
			pxa3xx_input_report_key(keypad, KEY_FOCUS, cam_key_status);
			af_flag = 0;
		}
		else
		{
			pxa3xx_input_report_key(keypad, KEY_CAMERA, cam_key_status);
		}

	}
	
	//printk("cam_key_status = %d\n", cam_key_status);
	//pxa3xx_input_report_key(keypad, KEY_CAMERA, cam_key_status);
}

static int cmmb_key_detect_handle(int irq, void *dev_id)
{
	int cmmb_key_status; //0:pressed, 1:unpressed
	
	cmmb_key_status = gpio_get_value(MOBILETV_KEY_GPIO);
	if(cmmb_key_status == 0)
		cmmb_key_status = 1024;
	else
		cmmb_key_status = 0;

	printk("cmmb_key_status = %d\n", cmmb_key_status);
	pxa3xx_input_report_key(keypad, KEY_MOBILETV, cmmb_key_status);
}

static int gpio_cmmb_key_init(struct pxa3xx_keypad *keypad)
{
	int err, cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(MOBILETV_KEY_GPIO);
	gpio_cd = MOBILETV_KEY_GPIO;

	/*
	 * setup GPIO for cmmb key
	 */
	err = gpio_request(gpio_cd, "cmmb key");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = request_irq(cd_irq, cmmb_key_detect_handle,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "cmmb key detect", keypad);
	if (err) {
		printk(KERN_ERR "%s: can't cmmb key detect IRQ\n", __func__);
		goto err_request_irq;
	}

	gpio_free(gpio_cd);
	return 0;

      err_request_irq:
	gpio_free(gpio_cd);
      err_request_cd:
	return err;
}
static int gpio_cam_key_init(struct pxa3xx_keypad *keypad)
{
	int err, cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(CAMERA_KEY_GPIO);
	gpio_cd = CAMERA_KEY_GPIO;

	/*
	 * setup GPIO for camera key
	 */
	err = gpio_request(gpio_cd, "camera key");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = request_irq(cd_irq, cam_key_detect_handle,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "camera key detect", keypad);
	if (err) {
		printk(KERN_ERR "%s: can't camera key detect IRQ\n", __func__);
		goto err_request_irq;
	}

	gpio_free(gpio_cd);
	return 0;

      err_request_irq:
	gpio_free(gpio_cd);
      err_request_cd:
	return err;
}

#endif


static int __init pxa3xx_keypad_probe(struct platform_device *pdev)
{
	//	struct pxa3xx_keypad *keypad;
	struct input_dev *input_dev;
	struct resource *res;
	int size;
	int ret = -EINVAL;

	printk( KERN_INFO "pxa3xx_keypad probe\n" );

	keypad = kzalloc(sizeof(struct pxa3xx_keypad), GFP_KERNEL);
	if (!keypad)
	      return -ENOMEM;

	keypad->pdata = pdev->dev.platform_data;
	if (keypad->pdata == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		ret = -EINVAL;
		goto failed0;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get register memory\n");
		return -ENXIO;
	}

	size = res->end - res->start + 1;
	keypad->mem = request_mem_region(res->start, size, pdev->name);
	if (keypad->mem == NULL) {
		dev_err(&pdev->dev, "failed to request register memory\n");
		ret = -EBUSY;
		goto failed0;
	}

	keypad->mmio_base = ioremap_nocache(res->start, size);
	if (keypad->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap registers\n");
		ret = -ENXIO;
		goto failed1;
	}
	keypad->clk = clk_get(&pdev->dev, "PXA3xx_KEYPAD");
	if (IS_ERR(keypad->clk)) {
		dev_err(&pdev->dev, "failed to get keypad clock\n");
		ret = PTR_ERR(keypad->clk);
		goto failed2;
	}
	keypad->lock = 0;

	/* setup input device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		ret = -ENOMEM;
		goto failed3;
	}
	input_dev->name = pdev->name;
	input_dev->id.bustype = BUS_HOST;
	input_dev->open  = pxa3xx_keypad_open;
	input_dev->close = pxa3xx_keypad_close;
	input_dev->private = keypad;

	keypad->input_dev = input_dev;
	dev_set_drvdata(&input_dev->dev, keypad);

	set_bit(EV_KEY, input_dev->evbit);
	//	set_bit(EV_REP, input_dev->evbit); 	Disable repeat key function
	set_bit(EV_REL, input_dev->evbit);

	pxa3xx_keypad_build_keycode(keypad);

	ret = input_register_device(input_dev);
	if (ret < 0) {
		input_free_device(input_dev);
		dev_err(&pdev->dev, "unable to register input device\n");
		goto failed3;
	}

	platform_set_drvdata(pdev, keypad);

	if(device_create_file(&input_dev->dev, &dev_attr_lock)) {
		dev_err(&pdev->dev, "Failed to add keypad lock attrs: %d\n", ret);
		goto failed4;
	}
#ifdef CONFIG_CPU_PXA930
	/* Enhanced Rotary Controller */
	if ((cpu_is_pxa930() || cpu_is_pxa935()) && keypad->pdata->enable_enhanced_rotary) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		size = res->end - res->start + 1;
		keypad->ercr_mem = request_mem_region(res->start, size, pdev->name);
		if (keypad->ercr_mem == NULL) {
			dev_err(&pdev->dev, "failed to request register memory\n");
			ret = -EBUSY;
			goto failed5;
		}
		keypad->ercr_base = ioremap_nocache(res->start, size);
		if (keypad->ercr_base == NULL) {
			dev_err(&pdev->dev, "failed to ioremap registers\n");
			ret = -ENXIO;
			goto failed6;
		}
		ret = request_irq(IRQ_ENHROT, enhanced_rotary_interrupt, 
					IRQF_DISABLED, "Enhanced Rotary", keypad);
		if (ret) {
			dev_err(&pdev->dev, "failed to request irq = IRQ_ENHROT ");
			goto failed7;
		}
	}
#endif
	ret = request_irq(IRQ_KEYPAD, pxa3xx_keypad_interrupt, IRQF_DISABLED, pdev->name, keypad);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq = IRQ_KEYPAD");
		goto failed8;
	}

#ifdef LC6830_INVENTEC_KEYPAD
	gpio_cam_key_init(keypad);
	gpio_cmmb_key_init(keypad);
#endif
	
#if defined(CONFIG_PXA3xx_DVFM)
	dvfm_register("Keypad", &dvfm_lock.dev_idx);
	dvfm_register_notifier(&notifier_freq_block,
				DVFM_FREQUENCY_NOTIFIER);
	init_timer(&kp_timer);
	kp_timer.function = keypad_timer_handler;
	kp_timer.data = 0;
#endif

	//fang for autofocus key
	init_timer(&cam_timer);
	cam_timer.function = cam_timer_handler;
	cam_timer.data = 0;

#ifdef CONFIG_TOUCH_KEYPAD
	touch_keypad = keypad;
#endif

	keyevent_enable = 1;

#if defined(CONFIG_KEYBOARD_PXA3xx_LOCK) || defined(CONFIG_ANDROID_POWER)
	android_register_late_suspend(&keypad_android_suspend);
#endif

	return 0;

failed8:
#ifdef CONFIG_CPU_PXA930
	free_irq(IRQ_ENHROT, keypad);
failed7:
	iounmap(keypad->ercr_base);
failed6:
	release_mem_region(keypad->ercr_mem->start,
				keypad->ercr_mem->end - keypad->ercr_mem->start + 1);
failed5:
#endif
	device_remove_file(&input_dev->dev, &dev_attr_lock);
failed4:
	input_unregister_device(input_dev);
failed3:
	clk_put(keypad->clk);
failed2:
	iounmap(keypad->mmio_base);
failed1:
	release_mem_region(keypad->mem->start,
				keypad->mem->end - keypad->mem->start + 1);
failed0:
	kfree(keypad);
	return ret;
}

static int __devexit pxa3xx_keypad_remove(struct platform_device *pdev)
{
	struct pxa3xx_keypad *keypad = platform_get_drvdata(pdev);

#if defined(CONFIG_KEYBOARD_PXA3xx_LOCK) || defined(CONFIG_ANDROID_POWER)
	android_unregister_late_suspend(&keypad_android_suspend);
#endif

#if defined(CONFIG_PXA3xx_DVFM)
	dvfm_unregister_notifier(&notifier_freq_block,
				DVFM_FREQUENCY_NOTIFIER);
	dvfm_unregister("Keypad", &dvfm_lock.dev_idx);
#endif
	clk_disable(keypad->clk);
	free_irq(IRQ_KEYPAD, keypad);
#ifdef CONFIG_CPU_PXA930
	free_irq(IRQ_ENHROT, keypad);
	iounmap(keypad->ercr_base);
	release_mem_region(keypad->ercr_mem->start,
				keypad->ercr_mem->end - keypad->ercr_mem->start + 1);
#endif
	device_remove_file(&keypad->input_dev->dev, &dev_attr_lock);
	input_unregister_device(keypad->input_dev);
	clk_put(keypad->clk);
	iounmap(keypad->mmio_base);
	release_mem_region(keypad->mem->start,
				keypad->mem->end - keypad->mem->start + 1);
	kfree(keypad);
	return 0;
}

static struct platform_driver pxa3xx_keypad_driver = {
	.driver		= {
		.name	= "pxa3xx-keypad",
	},
	.probe		= pxa3xx_keypad_probe,
	.remove		= __devexit_p(pxa3xx_keypad_remove),
	.suspend	= pxa3xx_keypad_suspend,
	.resume		= pxa3xx_keypad_resume,
};

static int __init pxa3xx_keypad_init(void)
{
#ifdef CONFIG_ANDROID_POWER
	pxa3xx_keypad_suspend_lock.name = "pxa3xx_keypad";
	android_init_suspend_lock(&pxa3xx_keypad_suspend_lock);
#endif
	return platform_driver_register(&pxa3xx_keypad_driver);
}

static void __exit pxa3xx_keypad_exit(void)
{
#ifdef CONFIG_ANDROID_POWER
	android_uninit_suspend_lock(&pxa3xx_keypad_suspend_lock);
#endif
	platform_driver_unregister(&pxa3xx_keypad_driver);
}

late_initcall(pxa3xx_keypad_init);
module_exit(pxa3xx_keypad_exit);
