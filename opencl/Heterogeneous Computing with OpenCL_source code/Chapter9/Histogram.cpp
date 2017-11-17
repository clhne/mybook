/* ============================================================

Copyright (c) 2010 Advanced Micro Devices, Inc.  All rights reserved.

Redistribution and use of this material is permitted under the following 
conditions:

Redistributions must retain the above copyright notice and all terms of this 
license.

In no event shall anyone redistributing or accessing or using this material 
commence or participate in any arbitration or legal action relating to this 
material against Advanced Micro Devices, Inc. or any copyright holders or 
contributors. The foregoing shall survive any expiration or termination of 
this license or any agreement or access or use related to this material. 

ANY BREACH OF ANY TERM OF THIS LICENSE SHALL RESULT IN THE IMMEDIATE REVOCATION 
OF ALL RIGHTS TO REDISTRIBUTE, ACCESS OR USE THIS MATERIAL.

THIS MATERIAL IS PROVIDED BY ADVANCED MICRO DEVICES, INC. AND ANY COPYRIGHT 
HOLDERS AND CONTRIBUTORS "AS IS" IN ITS CURRENT CONDITION AND WITHOUT ANY 
REPRESENTATIONS, GUARANTEE, OR WARRANTY OF ANY KIND OR IN ANY WAY RELATED TO 
SUPPORT, INDEMNITY, ERROR FREE OR UNINTERRUPTED OPERA TION, OR THAT IT IS FREE 
FROM DEFECTS OR VIRUSES.  ALL OBLIGATIONS ARE HEREBY DISCLAIMED - WHETHER 
EXPRESS, IMPLIED, OR STATUTORY - INCLUDING, BUT NOT LIMITED TO, ANY IMPLIED 
WARRANTIES OF TITLE, MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, 
ACCURACY, COMPLETENESS, OPERABILITY, QUALITY OF SERVICE, OR NON-INFRINGEMENT. 
IN NO EVENT SHALL ADVANCED MICRO DEVICES, INC. OR ANY COPYRIGHT HOLDERS OR 
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, PUNITIVE,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, REVENUE, DATA, OR PROFITS; OR 
BUSINESS INTERRUPTION) HOWEVER CAUSED OR BASED ON ANY THEORY OF LIABILITY 
ARISING IN ANY WAY RELATED TO THIS MATERIAL, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE. THE ENTIRE AND AGGREGATE LIABILITY OF ADVANCED MICRO DEVICES, 
INC. AND ANY COPYRIGHT HOLDERS AND CONTRIBUTORS SHALL NOT EXCEED TEN DOLLARS 
(US $10.00). ANYONE REDISTRIBUTING OR ACCESSING OR USING THIS MATERIAL ACCEPTS 
THIS ALLOCATION OF RISK AND AGREES TO RELEASE ADVANCED MICRO DEVICES, INC. AND 
ANY COPYRIGHT HOLDERS AND CONTRIBUTORS FROM ANY AND ALL LIABILITIES, 
OBLIGATIONS, CLAIMS, OR DEMANDS IN EXCESS OF TEN DOLLARS (US $10.00). THE 
FOREGOING ARE ESSENTIAL TERMS OF THIS LICENSE AND, IF ANY OF THESE TERMS ARE 
CONSTRUED AS UNENFORCEABLE, FAIL IN ESSENTIAL PURPOSE, OR BECOME VOID OR 
DETRIMENTAL TO ADVANCED MICRO DEVICES, INC. OR ANY COPYRIGHT HOLDERS OR 
CONTRIBUTORS FOR ANY REASON, THEN ALL RIGHTS TO REDISTRIBUTE, ACCESS OR USE 
THIS MATERIAL SHALL TERMINATE IMMEDIATELY. MOREOVER, THE FOREGOING SHALL 
SURVIVE ANY EXPIRATION OR TERMINATION OF THIS LICENSE OR ANY AGREEMENT OR 
ACCESS OR USE RELATED TO THIS MATERIAL.

NOTICE IS HEREBY PROVIDED, AND BY REDISTRIBUTING OR ACCESSING OR USING THIS 
MATERIAL SUCH NOTICE IS ACKNOWLEDGED, THAT THIS MATERIAL MAY BE SUBJECT TO 
RESTRICTIONS UNDER THE LAWS AND REGULATIONS OF THE UNITED STATES OR OTHER 
COUNTRIES, WHICH INCLUDE BUT ARE NOT LIMITED TO, U.S. EXPORT CONTROL LAWS SUCH 
AS THE EXPORT ADMINISTRATION REGULATIONS AND NATIONAL SECURITY CONTROLS AS 
DEFINED THEREUNDER, AS WELL AS STATE DEPARTMENT CONTROLS UNDER THE U.S. 
MUNITIONS LIST. THIS MATERIAL MAY NOT BE USED, RELEASED, TRANSFERRED, IMPORTED,
EXPORTED AND/OR RE-EXPORTED IN ANY MANNER PROHIBITED UNDER ANY APPLICABLE LAWS, 
INCLUDING U.S. EXPORT CONTROL LAWS REGARDING SPECIFICALLY DESIGNATED PERSONS, 
COUNTRIES AND NATIONALS OF COUNTRIES SUBJECT TO NATIONAL SECURITY CONTROLS. 
MOREOVER, THE FOREGOING SHALL SURVIVE ANY EXPIRATION OR TERMINATION OF ANY 
LICENSE OR AGREEMENT OR ACCESS OR USE RELATED TO THIS MATERIAL.

NOTICE REGARDING THE U.S. GOVERNMENT AND DOD AGENCIES: This material is 
provided with "RESTRICTED RIGHTS" and/or "LIMITED RIGHTS" as applicable to 
computer software and technical data, respectively. Use, duplication, 
distribution or disclosure by the U.S. Government and/or DOD agencies is 
subject to the full extent of restrictions in all applicable regulations, 
including those found at FAR52.227 and DFARS252.227 et seq. and any successor 
regulations thereof. Use of this material by the U.S. Government and/or DOD 
agencies is acknowledgment of the proprietary rights of any copyright holders 
and contributors, including those of Advanced Micro Devices, Inc., as well as 
the provisions of FAR52.227-14 through 23 regarding privately developed and/or 
commercial computer software.

This license forms the entire agreement regarding the subject matter hereof and 
supersedes all proposals and prior discussions and writings between the parties 
with respect thereto. This license does not affect any ownership, rights, title,
or interest in, or relating to, this material. No terms of this license can be 
modified or waived, and no breach of this license can be excused, unless done 
so in a writing signed by all affected parties. Each term of this license is 
separately enforceable. If any term of this license is determined to be or 
becomes unenforceable or illegal, such term shall be reformed to the minimum 
extent necessary in order for this license to remain in effect in accordance 
with its terms as modified by such reformation. This license shall be governed 
by and construed in accordance with the laws of the State of Texas without 
regard to rules on conflicts of law of any state or jurisdiction or the United 
Nations Convention on the International Sale of Goods. All disputes arising out 
of this license shall be subject to the jurisdiction of the federal and state 
courts in Austin, Texas, and all defenses are hereby waived concerning personal 
jurisdiction and venue of these courts.

============================================================ */

// Fast histogram computation using __local memory
// marc.romankewicz@amd.com

#include "Histogram.h"
#include "Timer.h"
#include <time.h>
    
#define NTIMERS 1
CPerfCounter t[NTIMERS];

const char *cluErrorString(cl_int err) {

   switch(err) {

      case CL_SUCCESS: return "CL_SUCCESS";
      case CL_DEVICE_NOT_FOUND: return "CL_DEVICE_NOT_FOUND";
      case CL_DEVICE_NOT_AVAILABLE: return "CL_DEVICE_NOT_AVAILABLE";
      case CL_COMPILER_NOT_AVAILABLE: return
                                       "CL_COMPILER_NOT_AVAILABLE";
      case CL_MEM_OBJECT_ALLOCATION_FAILURE: return
                                       "CL_MEM_OBJECT_ALLOCATION_FAILURE";
      case CL_OUT_OF_RESOURCES: return "CL_OUT_OF_RESOURCES";
      case CL_OUT_OF_HOST_MEMORY: return "CL_OUT_OF_HOST_MEMORY";
      case CL_PROFILING_INFO_NOT_AVAILABLE: return
                                       "CL_PROFILING_INFO_NOT_AVAILABLE";
      case CL_MEM_COPY_OVERLAP: return "CL_MEM_COPY_OVERLAP";
      case CL_IMAGE_FORMAT_MISMATCH: return "CL_IMAGE_FORMAT_MISMATCH";
      case CL_IMAGE_FORMAT_NOT_SUPPORTED: return
                                       "CL_IMAGE_FORMAT_NOT_SUPPORTED";
      case CL_BUILD_PROGRAM_FAILURE: return "CL_BUILD_PROGRAM_FAILURE";
      case CL_MAP_FAILURE: return "CL_MAP_FAILURE";
      case CL_INVALID_VALUE: return "CL_INVALID_VALUE";
      case CL_INVALID_DEVICE_TYPE: return "CL_INVALID_DEVICE_TYPE";
      case CL_INVALID_PLATFORM: return "CL_INVALID_PLATFORM";
      case CL_INVALID_DEVICE: return "CL_INVALID_DEVICE";
      case CL_INVALID_CONTEXT: return "CL_INVALID_CONTEXT";
      case CL_INVALID_QUEUE_PROPERTIES: return "CL_INVALID_QUEUE_PROPERTIES";
      case CL_INVALID_COMMAND_QUEUE: return "CL_INVALID_COMMAND_QUEUE";
      case CL_INVALID_HOST_PTR: return "CL_INVALID_HOST_PTR";
      case CL_INVALID_MEM_OBJECT: return "CL_INVALID_MEM_OBJECT";
      case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR: return
                                       "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR";
      case CL_INVALID_IMAGE_SIZE: return "CL_INVALID_IMAGE_SIZE";
      case CL_INVALID_SAMPLER: return "CL_INVALID_SAMPLER";
      case CL_INVALID_BINARY: return "CL_INVALID_BINARY";
      case CL_INVALID_BUILD_OPTIONS: return "CL_INVALID_BUILD_OPTIONS";
      case CL_INVALID_PROGRAM: return "CL_INVALID_PROGRAM";
      case CL_INVALID_PROGRAM_EXECUTABLE: return
                                       "CL_INVALID_PROGRAM_EXECUTABLE";
      case CL_INVALID_KERNEL_NAME: return "CL_INVALID_KERNEL_NAME";
      case CL_INVALID_KERNEL_DEFINITION: return "CL_INVALID_KERNEL_DEFINITION";
      case CL_INVALID_KERNEL: return "CL_INVALID_KERNEL";
      case CL_INVALID_ARG_INDEX: return "CL_INVALID_ARG_INDEX";
      case CL_INVALID_ARG_VALUE: return "CL_INVALID_ARG_VALUE";
      case CL_INVALID_ARG_SIZE: return "CL_INVALID_ARG_SIZE";
      case CL_INVALID_KERNEL_ARGS: return "CL_INVALID_KERNEL_ARGS";
      case CL_INVALID_WORK_DIMENSION: return "CL_INVALID_WORK_DIMENSION";
      case CL_INVALID_WORK_GROUP_SIZE: return "CL_INVALID_WORK_GROUP_SIZE";
      case CL_INVALID_WORK_ITEM_SIZE: return "CL_INVALID_WORK_ITEM_SIZE";
      case CL_INVALID_GLOBAL_OFFSET: return "CL_INVALID_GLOBAL_OFFSET";
      case CL_INVALID_EVENT_WAIT_LIST: return "CL_INVALID_EVENT_WAIT_LIST";
      case CL_INVALID_EVENT: return "CL_INVALID_EVENT";
      case CL_INVALID_OPERATION: return "CL_INVALID_OPERATION";
      case CL_INVALID_GL_OBJECT: return "CL_INVALID_GL_OBJECT";
      case CL_INVALID_BUFFER_SIZE: return "CL_INVALID_BUFFER_SIZE";
      case CL_INVALID_MIP_LEVEL: return "CL_INVALID_MIP_LEVEL";

      default: return "UNKNOWN CL ERROR CODE";
   }
}

std::string
convertToString(const char *filename)
{
    size_t size;
    char*  str;
    std::string s;

    std::fstream f(filename, (std::fstream::in | std::fstream::binary));

    if(f.is_open())
    {
        size_t fileSize;
        f.seekg(0, std::fstream::end);
        size = fileSize = f.tellg();
        f.seekg(0, std::fstream::beg);

        str = new char[size+1];
        if(!str)
        {
            f.close();
            return NULL;
        }

        f.read(str, fileSize);
        f.close();
        str[size] = '\0';
    
        s = str;
        
        return s;
    }
    return NULL;
}

void
initializeCL(void)
{
    cl_int status = 0;
    cl_platform_id platform;

    clGetPlatformIDs( 1, &platform, NULL );

    clGetDeviceIDs( platform,
                    CL_DEVICE_TYPE_GPU,
                    1,
                    &device,
                    NULL);

    context = clCreateContext( NULL,
                               1,
                               &device,
                               NULL, NULL, NULL);

    commandQueue = clCreateCommandQueue( context, 
                                         device, 
                                         0, 
                                         &status);

    if(status != CL_SUCCESS) 
    { 
       fprintf(stderr, "clCreateCommandQueue: %s\n", cluErrorString(status));
       exit(-1);
    }

    clGetDeviceInfo(device,
                    CL_DEVICE_MAX_WORK_GROUP_SIZE,
                    sizeof(size_t),
                    &DeviceMaxWorkGroupSize,
                    NULL);

    clGetDeviceInfo(device,
                    CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
                    sizeof(cl_uint),
                    &DeviceMaxWorkItemDimensions,
                    NULL);

    clGetDeviceInfo(device,
                    CL_DEVICE_MAX_WORK_ITEM_SIZES,
                    sizeof(size_t) * 3,
                    DeviceMaxWorkItemSizes,
                    NULL);

    clGetDeviceInfo(device,
                    CL_DEVICE_LOCAL_MEM_SIZE,
                    sizeof(cl_ulong),
                    &DeviceLocalMemSize,
                    NULL);

    clGetDeviceInfo(device,
                    CL_DEVICE_GLOBAL_MEM_SIZE,
                    sizeof(cl_ulong),
                    &DeviceGlobalMemSize,
                    NULL);

	clGetDeviceInfo(device,
                    CL_DEVICE_MAX_MEM_ALLOC_SIZE,
                    sizeof(cl_ulong),
                    &DeviceMaxMemAllocSize,
                    NULL);

    clGetDeviceInfo(device,
                    CL_DEVICE_MAX_CLOCK_FREQUENCY,
                    sizeof(cl_uint),
                    &DeviceMaxClockFrequency,
                    NULL);

    clGetDeviceInfo(device,
                    CL_DEVICE_MAX_COMPUTE_UNITS,
                    sizeof(cl_uint),
                    &DeviceMaxComputeUnits,
                    NULL);
}

void
initializeKernel(void)
{
    cl_int status = 0;

    const char *filename  = "Histogram_Kernels.cl";
    std::string sourceStr = convertToString(filename);
    const char *source    = sourceStr.c_str();
    size_t sourceSize[]    = { strlen(source) };

    program = clCreateProgramWithSource( context, 
                                         1, 
                                         &source,
                                         sourceSize,
                                         &status);
    if(status != CL_SUCCESS) 
    { 
       fprintf(stderr, "clCreateProgramWithSource: %s\n", 
                        cluErrorString(status));
       exit(-1);
    }

    status = clBuildProgram(program, 1, &device, "-I .", NULL, NULL);

    char buf[0x10000];

    clGetProgramBuildInfo( program,
                           device,
                           CL_PROGRAM_BUILD_LOG,
                           0x10000,
                           buf,
                           NULL);

    if(!simpleOutput)
		printf("%s\n", buf);

    if(status != CL_SUCCESS) 
    { 
         fprintf(stderr, "clBuildProgram: %s\n",
                          cluErrorString(status));
         exit(-1);
    }

    histogram = clCreateKernel(program, "histogramKernel", &status);

    if(status != CL_SUCCESS) 
    { 
       fprintf(stderr, "clCreateKernel: %s\n", cluErrorString(status));
       exit(-1);
    }

    reduce = clCreateKernel(program, "reduceKernel", &status);

    if(status != CL_SUCCESS) 
    { 
       fprintf(stderr, "clCreateKernel: %s\n", cluErrorString(status));
       exit(-1);
    }

    clGetKernelWorkGroupInfo(histogram,
                             device,
                             CL_KERNEL_LOCAL_MEM_SIZE,
                             sizeof(cl_ulong),
                             &KernelLocalMemSize,
                             NULL);

    clGetKernelWorkGroupInfo(histogram,
                             device,
                             CL_KERNEL_WORK_GROUP_SIZE,
                             sizeof(size_t),
                             &KernelWorkGroupSize,
                             NULL);

    clGetKernelWorkGroupInfo(histogram,
                             device,
                             CL_KERNEL_COMPILE_WORK_GROUP_SIZE,
                             sizeof(size_t) * 3,
                             KernelCompileWorkGroupSize,
                             NULL);
}

void
initializeCLBuffers(void)
{
    cl_int status = 0;
 
    input =  (void *) malloc( inputNBytes );
    output = (void *) malloc( outputNBytes );
    memset( (void *) input,  0, inputNBytes );
    memset( (void *) output, 0, outputNBytes );

    if(!uniformData)
	{

       // quick & dirty, portable, fast, MWC random init

       time_t ltime;
       time(&ltime);
       cl_uint a = (cl_uint) ltime, b = (cl_uint) ltime;

       cl_uint *p = (cl_uint *) input;

       for(unsigned int i=0; i < inputNBytes / sizeof(cl_uint); i++)
          p[i] = ( b = ( a * ( b & 65535 )) + (  b >> 16 ));
	}

    inputBuffer = clCreateBuffer( context, 
                                  CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                                  inputNBytes,
                                  input, 
                                  &status);
    if(status != CL_SUCCESS) 
    { 
       fprintf(stderr, "clCreateBuffer input: %s\n", cluErrorString(status));
       exit(-1);
    }

    outputBuffer = clCreateBuffer( context, 
                                   CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                                   outputNBytes,
                                   output, 
                                   &status);
    if(status != CL_SUCCESS) 
    { 
       fprintf(stderr, "clCreateBuffer output: %s\n", cluErrorString(status));
       exit(-1);
    }
}

void
dumpInfo(void)
{
    printf("nLoops             %d\n", nLoops);
    printf("n4Vectors          %d\n", n4Vectors);
    printf("nThreads           %d\n", nThreads);
    printf("nThreadsPerGroup   %d\n", nThreadsPerGroup);
    printf("n4VectorsPerThread %d\n", n4VectorsPerThread);
    printf("nGroups            %d\n", nGroups);
    printf("inputNBytes        %d\n", inputNBytes);
    printf("outputNBytes       %d\n", outputNBytes);
    printf("\n");

    printf("CL_DEVICE_MAX_WORK_GROUP_SIZE:      %d\n", 
            DeviceMaxWorkGroupSize);
    printf("CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS: %d\n",
            DeviceMaxWorkItemDimensions);
    printf("CL_DEVICE_MAX_WORK_ITEM_SIZES:      %d %d %d\n",
            DeviceMaxWorkItemSizes[0],
            DeviceMaxWorkItemSizes[1],
            DeviceMaxWorkItemSizes[2]);
    printf("CL_DEVICE_LOCAL_MEM_SIZE:           %d\n",
            DeviceLocalMemSize);
    printf("CL_DEVICE_GLOBAL_MEM_SIZE:          %d\n",
            DeviceGlobalMemSize);
	printf("CL_DEVICE_MAX_MEM_ALLOC_SIZE:       %d\n",
            DeviceMaxMemAllocSize);
    printf("CL_DEVICE_MAX_CLOCK_FREQUENCY:      %d\n", 
            DeviceMaxClockFrequency);
    printf("CL_DEVICE_MAX_COMPUTE_UNITS:        %d\n", 
            DeviceMaxComputeUnits);

    printf("\n");
    printf("CL_KERNEL_LOCAL_MEM_SIZE:           %d\n",
            KernelLocalMemSize);
    printf("CL_KERNEL_WORK_GROUP_SIZE:          %d\n",
            KernelWorkGroupSize);
    printf("CL_KERNEL_COMPILE_WORK_GROUP_SIZE:  %d %d %d\n",
            KernelCompileWorkGroupSize[0],
            KernelCompileWorkGroupSize[1],
            KernelCompileWorkGroupSize[2]);
}

void 
runCLKernels(void)
{
    cl_int   status;
    cl_event events[2];
    size_t globalThreads[3] = {1};
    size_t localThreads[3] = {1};
    size_t globalThreadsReduce = NBINS;
    size_t localThreadsReduce = 64;
    
    globalThreads[0] = nThreads;
    localThreads[0]  = nThreadsPerGroup;

	if(!simpleOutput)
	{
       printf("\n");
       printf("globalThreads[0] = %d\n", globalThreads[0] );
       printf("localThreads[0] =  %d\n", localThreads[0] );
	}

    status = clEnqueueWriteBuffer(
                commandQueue,
                inputBuffer,
                CL_TRUE,
                0,
                inputNBytes,
                input,
                0,
                NULL,
                &events[0]);
    
    if(status != CL_SUCCESS) 
    { 
       fprintf(stderr, "clEnqueueWriteBuffer: %s\n", cluErrorString(status));
       exit(-1);
    }

    int Arg=0;
    status = 0;
 
    // histogramKernel

    // __global input & output
    status |= clSetKernelArg(
                    histogram, 
                    Arg++, 
                    sizeof(cl_mem), 
                    (void *)&inputBuffer);

    status |= clSetKernelArg(
                    histogram, 
                    Arg++, 
                    sizeof(cl_mem), 
                    (void *)&outputBuffer);

    status |= clSetKernelArg(
                    histogram, 
                    Arg++, 
                    sizeof(n4VectorsPerThread), 
                    (void *)&n4VectorsPerThread);

    // reduceKernel

    Arg = 0;
    status |= clSetKernelArg(
                    reduce, 
                    Arg++, 
                    sizeof(cl_mem), 
                    (void *)&outputBuffer);

    status |= clSetKernelArg(
                    reduce, 
                    Arg++, 
                    sizeof(nGroups), 
                    (void *)&nGroups);

    if(status != CL_SUCCESS) 
    { 
        fprintf(stderr, "clSetKernelArg: %s\n", cluErrorString(status));
        exit(-1);
    }

    clFinish( commandQueue );

    if(!simpleOutput)
		printf("\nkernel loop .. ");
    t[0].Reset();
    t[0].Start();

    for( unsigned int i = 0; i < nLoops; i++ ) {

       status = clEnqueueNDRangeKernel( commandQueue,
                                        histogram,
                                        1,
                                        NULL,
                                        globalThreads,
                                        localThreads,
                                        0,
                                        NULL,
                                        NULL);

       if(status != CL_SUCCESS) 
       { 
          fprintf(stderr, "clEnqueueNDRangeKernel histogramKernel: %s\n",
                           cluErrorString(status));
          exit(-1);
       }

       status = clEnqueueNDRangeKernel( commandQueue,
                                        reduce,
                                        1,
                                        NULL,
                                        &globalThreadsReduce,
                                        &localThreadsReduce,
                                        0,
                                        NULL,
                                        NULL );

       if(status != CL_SUCCESS) 
       { 
          fprintf(stderr, "clEnqueueNDRangeKernel reduceKernel: %s\n",
                           cluErrorString(status));
          exit(-1);
       }
    }

    clFinish( commandQueue );
    t[0].Stop();
	if(!simpleOutput)
       printf("done\n");

    status = clEnqueueReadBuffer(
                commandQueue,
                outputBuffer,
                CL_TRUE,
                0,
                outputNBytes,
                output,
                0,
                NULL,
                &events[1]);
    
    if(status != CL_SUCCESS) 
    { 
       fprintf(stderr, "clEnqueueReadBuffer: %s\n", cluErrorString(status));
       exit(-1);
    }
    
    clReleaseEvent(events[0]);
    clReleaseEvent(events[1]);
}

int 
main(int argc, char * argv[])
{
    putenv("GPU_DUMP_DEVICE_KERNEL=3");

	while(--argc) {
	    if( strcmp(argv[argc], "-s") == 0 )
			simpleOutput=true;
		if( strcmp(argv[argc], "-u") == 0 )
			uniformData=true;
	}

    initializeCL();
    initializeKernel();

    nThreads =           64*1024;
    nThreadsPerGroup =   (cl_uint) KernelCompileWorkGroupSize[0];
    nGroups =            nThreads / nThreadsPerGroup;

    n4Vectors =          (cl_uint) ( DeviceMaxMemAllocSize / sizeof(cl_uint4) );
    n4VectorsPerThread = n4Vectors / nThreads;
    inputNBytes =        n4Vectors * sizeof(cl_uint4);
    outputNBytes =       nGroups * NBINS * sizeof(cl_uint);

    nLoops =             1000;

    initializeCLBuffers();

    if(!simpleOutput) 
		dumpInfo();

    // compute CPU histogram

    cl_int *p = (cl_int *) input;

    for(unsigned int i=0; i < inputNBytes / sizeof(cl_uint); i++) {

        cpuhist[ (p[i] >> 24) & 0xff ]++;
        cpuhist[ (p[i] >> 16) & 0xff ]++;
        cpuhist[ (p[i] >> 8) & 0xff ]++;
        cpuhist[ (p[i] >> 0) & 0xff ]++;
    }

    runCLKernels();

    gpuhist = (cl_uint *) output;

    int mismatch = 0;

    for(int i=0; i<NBINS; i++) {

#if 0
       printf("gpuhist[%3d] = %8d cpuhist = %8d\n", i, gpuhist[i], cpuhist[i]);
#endif
       if(gpuhist[i] != cpuhist[i]) mismatch = 1;
    }

	double kernelTime = t[0].GetElapsedTime();
	double perf = ((double) inputNBytes * nLoops) /  kernelTime / 1e9;

	if(simpleOutput)
	{
		printf("%d,%.2f\n", mismatch ? 0 : 1, perf);
	}
	else
	{
       printf("\n");

       if(mismatch)
          printf("CPU, GPU MISMATCH\n");
       else
          printf("CPU, GPU MATCH\n");

       printf("\n");

       printf("kernelTime %.2fs B/W %.2f GPix/sec at 8 bits per pixel\n\n",
                                             kernelTime, perf);
	}

    return 0;
}
