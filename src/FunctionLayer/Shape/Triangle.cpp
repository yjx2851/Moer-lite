#include "Triangle.h"
#include <FunctionLayer/Acceleration/Linear.h>
//--- Triangle ---
Triangle::Triangle(int _primID, int _vtx0Idx, int _vtx1Idx, int _vtx2Idx,
                   const TriangleMesh *_mesh)
    : primID(_primID), vtx0Idx(_vtx0Idx), vtx1Idx(_vtx1Idx), vtx2Idx(_vtx2Idx),
      mesh(_mesh) {
  Point3f vtx0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]),
          vtx1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]),
          vtx2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);
  boundingBox.Expand(vtx0);
  boundingBox.Expand(vtx1);
  boundingBox.Expand(vtx2);
  this->geometryID = mesh->geometryID;
}

bool Triangle::rayIntersectShape(Ray &ray, int *primID, float *u,
                                 float *v) const {
  //* todo 实现三角形与光线求交
  Point3f origin = ray.origin;
  Vector3f direction = ray.direction;

  // 三角形三个顶点
  Point3f v0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]);
  Point3f v1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]);
  Point3f v2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);

  Vector3f edge1 = v1 - v0;
  Vector3f edge2 = v2 - v0;

  Vector3f pvec = cross(direction, edge2);
  float det = dot(edge1, pvec);

  // 允许双面交点
  if (fabs(det) < 1e-6f) return false;

  float invDet = 1.0f / det;
  Vector3f tvec = origin - v0;

  float uu = dot(tvec, pvec) * invDet;
  if (uu < 0.0f || uu > 1.0f) return false;

  Vector3f qvec = cross(tvec, edge1);
  float vv = dot(direction, qvec) * invDet;
  if (vv < 0.0f || (uu + vv) > 1.0f) return false;

  float t = dot(edge2, qvec) * invDet;
  if (t < ray.tNear || t > ray.tFar) return false;

  // 更新光线最近命中点
  ray.tFar = t;
  if (primID) *primID = this->primID;
  if (u) *u = uu;
  if (v) *v = vv;
  return true;
}

void Triangle::fillIntersection(float distance, int primID, float u, float v,
                                Intersection *intersection) const {
  // 该函数实际上不会被调用
  return;
}

//--- TriangleMesh ---
TriangleMesh::TriangleMesh(const Json &json) : Shape(json) {
  const auto &filepath = fetchRequired<std::string>(json, "file");
  meshData = MeshData::loadFromFile(filepath);
}

RTCGeometry TriangleMesh::getEmbreeGeometry(RTCDevice device) const {
  RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

  float *vertexBuffer = (float *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float),
      meshData->vertexCount);
  for (int i = 0; i < meshData->vertexCount; ++i) {
    Point3f vertex = transform.toWorld(meshData->vertexBuffer[i]);
    vertexBuffer[3 * i] = vertex[0];
    vertexBuffer[3 * i + 1] = vertex[1];
    vertexBuffer[3 * i + 2] = vertex[2];
  }

  unsigned *indexBuffer = (unsigned *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
      3 * sizeof(unsigned), meshData->faceCount);
  for (int i = 0; i < meshData->faceCount; ++i) {
    indexBuffer[i * 3] = meshData->faceBuffer[i][0].vertexIndex;
    indexBuffer[i * 3 + 1] = meshData->faceBuffer[i][1].vertexIndex;
    indexBuffer[i * 3 + 2] = meshData->faceBuffer[i][2].vertexIndex;
  }
  rtcCommitGeometry(geometry);
  return geometry;
}

bool TriangleMesh::rayIntersectShape(Ray &ray, int *primID, float *u,
                                     float *v) const {
  //* 当使用embree加速时，该方法不会被调用
  int geomID = -1;
  return acceleration->rayIntersect(ray, &geomID, primID, u, v);
}

void TriangleMesh::fillIntersection(float distance, int primID, float u,
                                    float v, Intersection *intersection) const {
  //* todo 填充光线与三角网格求交得到的交点信息
  intersection->distance = distance;
  intersection->shape = this;
  //* 1. 在三角形内部用插值计算交点坐标
  //* 2. 在三角形内部用插值计算法线
  //* 3. 在三角形内部用插值计算纹理坐标
  //* 4. 在三角形内部用插值计算交点的切线和副切线

  const auto &face = meshData->faceBuffer[primID];
  float w = 1.0f - u - v;

  // 顶点坐标插值
  int idx0 = face[0].vertexIndex;
  int idx1 = face[1].vertexIndex;
  int idx2 = face[2].vertexIndex;
  const Point3f &v0 = meshData->vertexBuffer[idx0];
  const Point3f &v1 = meshData->vertexBuffer[idx1];
  const Point3f &v2 = meshData->vertexBuffer[idx2];
  intersection->position =  v0 * w +  v1 * u +  v2 * v;
  
  // 法线插值
  int normalIndex0 = face[0].normalIndex;
  int normalIndex1 = face[1].normalIndex;
  int normalIndex2 = face[2].normalIndex;
  const Vector3f &n0 = meshData->normalBuffer[normalIndex0];
  const Vector3f &n1 = meshData->normalBuffer[normalIndex1];
  const Vector3f &n2 = meshData->normalBuffer[normalIndex2];
  intersection->normal =normalize(n0 * w + n1 * u + n2 * v);

  // 纹理坐标插值
  int texcodIndex0 = face[0].texcodIndex; 
  int texcodIndex1 = face[1].texcodIndex;
  int texcodIndex2 = face[2].texcodIndex;
  const Vector2f &uv0 = meshData->texcodBuffer[texcodIndex0];
  const Vector2f &uv1 = meshData->texcodBuffer[texcodIndex1];
  const Vector2f &uv2 = meshData->texcodBuffer[texcodIndex2];
  intersection->texCoord = uv0 * w + uv1 * u + uv2 * v;

  // 切线和副切线插值
  Vector3f edge1=v1-v0;
  Vector3f edge2=v2-v0;
  Vector2f deltaUV1 = uv1 - uv0;
  Vector2f deltaUV2 = uv2 - uv0;
  float f = 1.0f / (deltaUV1[0] * deltaUV2[1] - deltaUV2[0] * deltaUV1[1]);
  intersection->tangent = f * (deltaUV2[1] * edge1 - deltaUV1[1] * edge2);
  intersection->bitangent = f * (-deltaUV2[0] * edge1 + deltaUV1[0] * edge2);
  return;
}

void TriangleMesh::initInternalAcceleration() {
  acceleration = Acceleration::createAcceleration();
  int primCount = meshData->faceCount;
  for (int primID = 0; primID < primCount; ++primID) {
    int vtx0Idx = meshData->faceBuffer[primID][0].vertexIndex,
        vtx1Idx = meshData->faceBuffer[primID][1].vertexIndex,
        vtx2Idx = meshData->faceBuffer[primID][2].vertexIndex;
    std::shared_ptr<Triangle> triangle =
        std::make_shared<Triangle>(primID, vtx0Idx, vtx1Idx, vtx2Idx, this);
    acceleration->attachShape(triangle);
  }
  acceleration->build();
  // TriangleMesh的包围盒就是其内部加速结构的包围盒
  boundingBox = acceleration->boundingBox;
}
REGISTER_CLASS(TriangleMesh, "triangle")