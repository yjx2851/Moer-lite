#include "BVH.h"
struct  BVH::BVHNode{
    //* todo BVH节点结构设计
    BVHNode* left;
    BVHNode* right;
    AABB box;
    int firstShapeOffset = 0;
    int nShape;
    int splitAxis;
};

struct BVH::BVHShapeInfo {
    int id;
    AABB box;
    Point3f centroid;
};

void BVH::build() {
    AABB sceneBox;
    for (const auto & shape : shapes) {
        //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
        //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
        //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
        //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
        shape->initInternalAcceleration();
        boundingBox.Expand(shape->getAABB());
    }
    //* todo 完成BVH构建
    std::vector<BVHShapeInfo> shapeInfos;
    for (int i = 0; i < shapes.size(); ++i) {
        AABB bbox = shapes[i]->getAABB();
        Point3f centroid = bbox.Center();
        shapeInfos.push_back({i, bbox, centroid});
    }

    std::vector<std::shared_ptr<Shape>> orderedShapes;
    root = recursiveBuild(shapeInfos, 0, shapeInfos.size(), orderedShapes);

    shapes = orderedShapes;
}

BVH::BVHNode* BVH::recursiveBuild(std::vector<BVHShapeInfo>& shapeInfos, int l, int r, std::vector<std::shared_ptr<Shape>>& orderedShapes) {
    auto node = new BVHNode();

    if (r - l <= bvhLeafMaxSize) {
        node->firstShapeOffset = orderedShapes.size();
        node->nShape = r - l;
        for (int i = l; i < r; ++i) {
            orderedShapes.push_back(shapes[shapeInfos[i].id]);
        }
        AABB box;
        for (int i = l; i < r; ++i) {
            box.Expand(shapeInfos[i].box);
        }
        node->box = box;
        return node;
    }

    AABB centroidBounds;
    for (int i = l; i < r; ++i) {
        centroidBounds.Expand(shapeInfos[i].centroid);
    }

    int dim = centroidBounds.MaximumExtent();
    node->splitAxis = dim;

    float mid = 0.5f * (centroidBounds.pMin[dim] + centroidBounds.pMax[dim]);

    auto midIter = std::partition(shapeInfos.begin() + l, shapeInfos.begin() + r,
        [dim, mid](const BVHShapeInfo& info) {
            return info.centroid[dim] < mid;
        });
    int midIndex = midIter - shapeInfos.begin();

    if (midIndex == l || midIndex == r) {
        midIndex = (l + r) / 2;
    }

    node->left = recursiveBuild(shapeInfos, l, midIndex, orderedShapes);
    node->right = recursiveBuild(shapeInfos, midIndex, r, orderedShapes);

    node->box = node->left->box.Union(node->right->box);

    return node;
}




bool BVH::rayIntersect(Ray &ray, int *geomID, int *primID, float *u, float *v) const {
    //* todo 完成BVH求交
    // return false;
    // 辅助的递归函数
    
    std::function<bool(BVHNode*, Ray&)> intersectNode = [&](BVHNode* node, Ray& ray) -> bool {
        if (!node) {
            return false;
        }
        // 如果光线和当前节点包围盒不相交，直接跳过
        if (!node->box.RayIntersect(ray)) {
            return false;
        }
        bool hit = false;

        // 如果是叶子节点
        if (node->nShape > 0) {
            for (int i = 0; i < node->nShape; ++i) {
                auto shape = shapes[node->firstShapeOffset + i];
                if (shape->rayIntersectShape(ray, primID, u, v)) {
                    *geomID = shape->geometryID;
                    hit = true;
                }
            }
        } else { // 内部节点
            BVHNode *first, *second;
            if (ray.direction[node->splitAxis] > 0) {
                first = node->left;
                second = node->right;
            } else {
                first = node->right;
                second = node->left;
            }

            if (intersectNode(first, ray)) hit = true;
            if (intersectNode(second, ray)) hit = true;
        }
        return hit;
    };

    return intersectNode(root, ray);
}


