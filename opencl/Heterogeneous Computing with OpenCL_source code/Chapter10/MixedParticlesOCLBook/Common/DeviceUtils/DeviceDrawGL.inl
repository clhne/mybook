/*
		2011 Takahiro Harada
*/
#include <common/Math/Quaternion.h>
#include <common/Math/Matrix3x3.h>

__inline
void glVertexFloat4( const float4& v )
{
	glVertex3f( v.x, v.y, v.z );
}


__inline
void drawLine(const float4& a, const float4& b, const float4& color)
{
	glColor3fv( (float*)&color );
	glBegin(GL_LINES);
	glVertexFloat4( a );
	glVertexFloat4( b );
	glEnd();
}

__inline
void drawLineList(float4* vtx, u32* idx, int nVtx, int nIdx, const float4& color)
{
	glColor3fv( (float*)&color );
	glBegin(GL_LINES);
	for(int i=0; i<nIdx; i++)
	{
		glVertexFloat4( vtx[idx[i]] );
	}
	glEnd();
}

__inline
void drawPoint(const float4& a, const float4& color)
{
	glColor3fv( (float*)&color );
	glBegin(GL_POINTS);
	glVertexFloat4( a );
	glEnd();
}

__inline
void drawPointList(float4* vtx, const float4* color, int nVtx)
{
	glBegin(GL_POINTS);
	for(int i=0; i<nVtx; i++)
	{
		const float4& c = color[i];
		glColor4f(c.x, c.y, c.x, 1.f );
		glVertexFloat4( vtx[i] );
	}
	glEnd();
}

__inline
void drawPointListTransformed(const float4* vtx, const float4* color, int nVtx, const float4& translation, const Quaternion& quat)
{
	glPushMatrix();

	Matrix3x3 rotMat = mtTranspose( qtGetRotationMatrix( quat ) );
	float transformMat[16] =
	{
		rotMat.m_row[0].x, rotMat.m_row[0].y, rotMat.m_row[0].z, 0,
		rotMat.m_row[1].x, rotMat.m_row[1].y, rotMat.m_row[1].z, 0,
		rotMat.m_row[2].x, rotMat.m_row[2].y, rotMat.m_row[2].z, 0,
		translation.x, translation.y, translation.z,1
	};

	glMultMatrixf( transformMat );


	glBegin(GL_POINTS);
	for(int i=0; i<nVtx; i++)
	{
		const float4& c = color[i];
		glColor4f(c.x, c.y, c.z, 1);
		glVertexFloat4( vtx[i] );
	}
	glEnd();

	glPopMatrix();
}

__inline
void drawTriangle(const float4& a, const float4& b, const float4& c, const float4& color)
{
	glColor3fv( (float*)&color );
	glBegin(GL_TRIANGLES);
	glVertexFloat4( a );
	glVertexFloat4( b );
	glVertexFloat4( c );
	glEnd();
}

__inline
void drawTriangleList(float4* vtx, u32* idx, int nVtx, int nIdx, const float4& color)
{
	glColor3fv( (float*)&color );
	glBegin(GL_TRIANGLES);
	for(int i=0; i<nIdx; i++)
	{
		glVertexFloat4( vtx[ idx[i] ] );
	}
	glEnd();
}

__inline
void drawTriangleList1(float4* vtx, u32* idx, int nVtx, int nIdx, const float4* color)
{
	glBegin(GL_TRIANGLES);
	for(int i=0; i<nIdx; i++)
	{
		glColor3fv( (float*)&color[ idx[i] ] );
		glVertexFloat4( vtx[ idx[i] ] );
	}
	glEnd();
}

__inline
void drawTriangleList(const float4* vtx, const float4* vtxNormal, u32* idx, int nVtx, int nIdx, const float4& color)
{
	glColor3fv( (float*)&color );
	glBegin(GL_TRIANGLES);
	for(int i=0; i<nIdx; i++)
	{
		glNormal3f( vtxNormal[idx[i]].x, vtxNormal[idx[i]].y, vtxNormal[idx[i]].z );
		glVertexFloat4( vtx[ idx[i] ] );
	}
	glEnd();
}

__inline
void drawTriangleListTransformed(const float4* vtx, const float4* vtxNormal, u32* idx, int nVtx, int nIdx, const float4& color, const float4& translation, const Quaternion& quat)
{
	glPushMatrix();

	Matrix3x3 rotMat = mtTranspose( qtGetRotationMatrix( quat ) );
	float transformMat[16] =
	{
		rotMat.m_row[0].x, rotMat.m_row[0].y, rotMat.m_row[0].z, 0,
		rotMat.m_row[1].x, rotMat.m_row[1].y, rotMat.m_row[1].z, 0,
		rotMat.m_row[2].x, rotMat.m_row[2].y, rotMat.m_row[2].z, 0,
		translation.x, translation.y, translation.z,1
	};

	glMultMatrixf( transformMat );

	glColor3fv( (float*)&color );
	glBegin(GL_TRIANGLES);
	for(int i=0; i<nIdx; i++)
	{
		glNormal3f( vtxNormal[idx[i]].x, vtxNormal[idx[i]].y, vtxNormal[idx[i]].z );
		glVertexFloat4( vtx[ idx[i] ] );
	}
	glEnd();

	glPopMatrix();
}

__inline
void glDraw3DStrings(const char* str, const float4& pos)
{
	glRasterPos3f(pos.x, pos.y, pos.z);

	for(const char* c = str; *c!='\0'; c++)
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_12, *c );
}

__inline
void glDraw3DStrings(float value, const float4& pos)
{
	glDisable(GL_LIGHTING);

	char valueChar[128];
	sprintf_s(valueChar, "%3.2f", value);
	glDraw3DStrings(valueChar, pos);

	glEnable(GL_LIGHTING);
}

__inline
void drawAabb(const Aabb& a, const float4& color)
{

}
