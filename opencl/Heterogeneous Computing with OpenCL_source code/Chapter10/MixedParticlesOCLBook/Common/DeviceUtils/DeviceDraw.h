/*
		2011 Takahiro Harada
*/
#ifndef DEVICE_DRAW_H
#define DEVICE_DRAW_H

#include <Common/Math/Math.h>
#include <Common/DeviceUtils/DeviceUtils.h>
#include <Common/Geometry/Aabb.h>

#include <glut.h>
#include <Common/DeviceUtils/DeviceDrawGL.inl>
#define pxDrawLine(a,b,color) drawLine(a, b, color)
#define pxDrawLineList(vtx,idx,nVtx,nIdx,color) drawLineList(vtx,idx,nVtx,nIdx,color)
#define pxDrawPoint(a,color) drawPoint(a, color)
#define pxDrawPointList(vtx,color,nVtx) drawPointList(vtx,color,nVtx);
#define pxDrawPointSprite(vtx,color,radius, nVtx) drawPointList(vtx,color,nVtx)
#define pxDrawPointListTransformed(vtx,color,nVtx,translation,quaternion) drawPointListTransformed(vtx,color,nVtx,translation,quaternion);
#define pxDrawTriangle(a,b,c,color) drawTriangle(a,b,c,color)
#define pxDrawTriangleList(vtx,idx,nVtx,nIdx,color) drawTriangleList(vtx,idx,nVtx,nIdx,color)
#define pxDrawTriangleList1(vtx,idx,nVtx,nIdx,color) drawTriangleList1(vtx,idx,nVtx,nIdx,color)
#define pxDrawTriangleListNormal(vtx,vtxNormal,idx,nVtx,nIdx,color) drawTriangleList(vtx,vtxNormal,idx,nVtx,nIdx,color)
#define pxDrawTriangleListTessellated(vtx,vtxNormal,idx,nVtx,nIdx,color,translation,quaternion,vtxShader,hShader,dShader,pShader) drawTriangleListTransformed(vtx,vtxNormal,idx,nVtx,nIdx,color,translation,quaternion)
#define pxDrawTriangleListTransformed(vtx,vtxNormal,idx,nVtx,nIdx,color,translation,quaternion) drawTriangleListTransformed(vtx,vtxNormal,idx,nVtx,nIdx,color,translation,quaternion)

#define pxDrawText(txt,pos) glDraw3DStrings(txt, pos)
#define pxDrawAabb(aabb, c) drawAabb(aabb, c)

#define DevicePSShader int
#define pxCreatePixelShader(deviceData, shaderPath, profile, shaderOut) {shaderOut;}
#define pxDeleteShader(shader) {shader;}
#define pxSetPixelShader(pShader) {pShader;}

#define pxClearDepthStencil glClear( GL_DEPTH_BUFFER_BIT )

#endif

