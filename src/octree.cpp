#include <nori/octree.h>
#include <Eigen/Geometry>
#include <nori/common.h>
#include <vector>
#include <list>

NORI_NAMESPACE_BEGIN

const int OCTREE_CHILD_COUNT = 8;
const int OCTREE_LEAF_TRIANGLES = 10;
const float_t OCTREE_LEAF_MINBOUNDWIDTH = 1.0f / 32.f; //1.0f / 256.0f;// 2.0f;
float_t distanceInfinite = 64; //0.25f;// 1 << 16; // FLT_MAX / 2 - 100;

enum OCTreeFace {
    LeftBottomOuter,
    LeftTopOutter,
    RightTopOutter,
    RightBottomOutter,
    LeftBottomInner,
    LeftTopInner,
    RightTopInner,
    RightBottomInner,
};


enum OCTreeTraversal
{
    EachNode,
    OrderedNodeByDistance,
};

struct Triangle
{
public:
    bool overlaps(Mesh* mesh, BoundingBox3f& bbox)
    {
        mesh->getTriangleVertices(index, tempPoint0, tempPoint1, tempPoint2);
        tempBoundingBox3f.set(tempPoint0, tempPoint0);
        tempBoundingBox3f.expandBy(tempPoint1);
        tempBoundingBox3f.expandBy(tempPoint2);
        
        return bbox.overlaps(tempBoundingBox3f);
    }

public:
    uint32_t index;

private:
    static Point3f tempPoint0;
    static Point3f tempPoint1;
    static Point3f tempPoint2;
    static BoundingBox3f tempBoundingBox3f;
};

Point3f Triangle::tempPoint0;
Point3f Triangle::tempPoint1;
Point3f Triangle::tempPoint2;
BoundingBox3f Triangle::tempBoundingBox3f;

class OCTreeNode
{
public:
    OCTreeNode()
    {
        for (int i = 0; i < OCTREE_CHILD_COUNT; i++)
        {
            childs[i] = nullptr;
        }
        bbox.reset();
        triangles = nullptr;
    }

    ~OCTreeNode()
    {
        //delete[] childs; // invoke child's destructor
    }

public:
    OCTreeNode* childs[OCTREE_CHILD_COUNT];

    BoundingBox3f bbox;

    std::vector<Triangle*>* triangles;
    
};

class OCTreeNodeLeaf : public OCTreeNode
{
public:
    std::vector<Triangle*>* triangles;
};

struct OCTreeNodeDistance
{
    int index;
    float distance;
};

struct OCTreeNodeDistanceSort
{
    bool operator()(OCTreeNodeDistance left, OCTreeNodeDistance right) const {
        return left.distance < right.distance;
    }
} ocTreeNodeDistanceSort;

BoundingBox3f bboxInfinite(Point3f(-distanceInfinite, -distanceInfinite, -distanceInfinite), Point3f(distanceInfinite, distanceInfinite, distanceInfinite));
BoundingBox3f bboxTemp = bboxInfinite;

uint32_t countInteriorNode = 0;
uint32_t countLeafNode = 0;
uint32_t trianglesInLeafNodes = 0;
float averageTrianglesInLeafNodes = 0.0f;

OCTreeTraversal traversal = OCTreeTraversal::EachNode;

BoundingBox3f getBoundingBox(BoundingBox3f& bbox, int face)
{
    Point3f center = bbox.getCenter();
    Point3f extents = bbox.getExtents();
    if (face == (int)OCTreeFace::LeftBottomOuter)
    {
        Point3f min(center.x() - extents.x() / 2, center.y() - extents.y() / 2, center.z());
        Point3f max(center.x(), center.y(), center.z() + extents.z() / 2);
        bboxTemp.set(min, max);
    }
    else if (face == (int)OCTreeFace::LeftTopOutter)
    {
        Point3f min(center.x() - extents.x() / 2, center.y(), center.z());
        Point3f max(center.x(), center.y() + extents.y() / 2, center.z() + extents.z() / 2);
        bboxTemp.set(min, max);
    }
    else if (face == (int)OCTreeFace::RightTopOutter)
    {
        Point3f min(center.x(), center.y(), center.z());
        Point3f max(center.x() + extents.x() / 2, center.y() + extents.y() / 2, center.z() + extents.z() / 2);
        bboxTemp.set(min, max);
    }
    else if (face == (int)OCTreeFace::RightBottomOutter)
    {
        Point3f min(center.x(), center.y() - extents.y() / 2, center.z());
        Point3f max(center.x() + extents.x() / 2, center.y(), center.z() + extents.z() / 2);
        bboxTemp.set(min, max);
    }
    else if (face == (int)OCTreeFace::LeftBottomInner)
    {
        Point3f min(center.x() - extents.x() / 2, center.y() - extents.y() / 2, center.z() - extents.z() / 2);
        Point3f max(center.x(), center.y(), center.z());
        bboxTemp.set(min, max);
    }
    else if (face == (int)OCTreeFace::LeftTopInner)
    {
        Point3f min(center.x() - extents.x() / 2, center.y(), center.z() - extents.z() / 2);
        Point3f max(center.x(), center.y() + extents.y() / 2, center.z());
        bboxTemp.set(min, max);
    }
    else if (face == (int)OCTreeFace::RightTopInner)
    {
        Point3f min(center.x(), center.y(), center.z() - extents.z() / 2);
        Point3f max(center.x() + extents.x() / 2, center.y() + extents.y() / 2, center.z());
        bboxTemp.set(min, max);
    }
    else
    {
        Point3f min(center.x(), center.y() - extents.y() / 2, center.z() - extents.z() / 2);
        Point3f max(center.x() + extents.x() / 2, center.y(), center.z());
        bboxTemp.set(min, max);
    }

    return bboxTemp;
}

bool needTerminate(BoundingBox3f& bbox, std::vector<Triangle*>* triangles)
{
    size_t triangleCount = triangles->size();
    if (triangleCount <= OCTREE_LEAF_TRIANGLES)
        return true;

    if (bbox.getExtents().x() <= OCTREE_LEAF_MINBOUNDWIDTH)
        return true;

    return false;
}

OCTreeNode* buildRecursively(Mesh* mesh, BoundingBox3f& bbox, std::vector<Triangle*>* triangles)
{
    if (needTerminate(bbox, triangles))
    {
        size_t triangleCount = triangles->size();
        if (triangleCount > 0)
        {
            OCTreeNode* nodeLeaf = new OCTreeNode();
            nodeLeaf->triangles = new std::vector<Triangle *>;
            for (auto iterator = triangles->begin(); iterator != triangles->end(); iterator++)
                nodeLeaf->triangles->push_back(*iterator);
            
            countLeafNode++;
            trianglesInLeafNodes += (uint32_t)triangleCount;

            return nodeLeaf;
        }
        else
        {
            return nullptr;
        }
    }

    // split to eight child node
    std::vector<Triangle*> splitedTriangles[OCTREE_CHILD_COUNT];
    for (auto iterator = triangles->begin(); iterator != triangles->end(); iterator++)
    {
        Triangle* triangle = *iterator;
        for (int i = 0; i < OCTREE_CHILD_COUNT; i++)
        {
            BoundingBox3f bboxChild = getBoundingBox(bbox, i);
            if (triangle->overlaps(mesh, bboxChild))
            {
                splitedTriangles[i].push_back(triangle);
            }
        }
    }

    // buildRecursively
    OCTreeNode* node = new OCTreeNode();
    for (int i = 0; i < OCTREE_CHILD_COUNT; i++)
    {
        BoundingBox3f bboxChild = getBoundingBox(bbox, i);
        OCTreeNode* child = buildRecursively(mesh, bboxChild, &splitedTriangles[i]);
        if (child == nullptr)
            continue;

        node->childs[i] = child;

        child->bbox = bboxChild;
    }
    countInteriorNode++;
    return node;
}

AccelOCTree::~AccelOCTree()
{
    delete rootNode;
}

void AccelOCTree::build()
{
    //Accel::build();
    // build an octree

    std::vector<Triangle*> triangles;
    //std::vector<Triangle> triangles;
    for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
        Triangle* triangle = new Triangle();
        triangle->index = idx;

        triangles.push_back(triangle);
    }
    //Point3f extents = m_mesh->getBoundingBox().getExtents();
    BoundingBox3f bbox = bboxInfinite; // default is infinite Bounding Box
    rootNode = buildRecursively(m_mesh, bbox, &triangles);
    rootNode->bbox = bbox;
    averageTrianglesInLeafNodes = (float)trianglesInLeafNodes / countLeafNode;
    std::cout << "\n" << countInteriorNode << ", " << countLeafNode << ", " << trianglesInLeafNodes << ", " << averageTrianglesInLeafNodes;
}

bool rayIntersectRecursivelyUnorder(Mesh* mesh, OCTreeNode* node, Ray3f &ray, Intersection &its, uint32_t& faceIndex, bool shadowRay)
{
    if (!node->bbox.rayIntersect(ray))
        return false;

    bool foundIntersection = false;  // Was an intersection found so far?

    // leaf node
    if (node->triangles != nullptr)
    {
        for (auto iterator = node->triangles->begin(); iterator != node->triangles->end(); iterator++)
        {
            uint32_t index = (*iterator)->index;
            float u, v, t;
            if (mesh->rayIntersect(index, ray, u, v, t))
            {
                if (shadowRay)
                    return true;
                
                ray.maxt = its.t = t; // make next intersection is closer than maxt
                its.uv = Point2f(u, v);
                its.mesh = mesh;

                faceIndex = index;
                foundIntersection = true;
            }
        }
        return foundIntersection;
    }

    // interior node
    for (int i = 0; i < OCTREE_CHILD_COUNT; i++)
    {
        OCTreeNode* child = node->childs[i];
        if (child == NULL)
            continue;

        if (!rayIntersectRecursivelyUnorder(mesh, child, ray, its, faceIndex, shadowRay))
            continue;
        foundIntersection = true;

        if (shadowRay)
            return true;
    }

    return foundIntersection;
}

bool rayIntersectRecursivelyInOrder(Mesh* mesh, OCTreeNode* node, Ray3f &ray, Intersection &its, uint32_t& faceIndex, bool shadowRay)
{
    if (!node->bbox.rayIntersect(ray))
        return false;

    bool foundIntersection = false;  // Was an intersection found so far?

    // leaf node
    if (node->triangles != nullptr)
    {
        for (auto iterator = node->triangles->begin(); iterator != node->triangles->end(); iterator++)
        {
            uint32_t index = (*iterator)->index;
            float u, v, t;
            //countRayIntersect++;
            if (mesh->rayIntersect(index, ray, u, v, t))
            {
                if (shadowRay)
                    return true;

                ray.maxt = its.t = t; // make next intersection is closer than maxt
                its.uv = Point2f(u, v);
                its.mesh = mesh;

                faceIndex = index;
                foundIntersection = true;
            }
        }
        return foundIntersection;
    }

    // interior node
    //// https://www.cnblogs.com/lizhenghao126/p/11053598.html
    //// https://cloud.tencent.com/developer/ask/34249
    //// 1. sort child node by distance from ray to intersection
    OCTreeNodeDistance distances[OCTREE_CHILD_COUNT];
    for (int i = 0; i < OCTREE_CHILD_COUNT; i++)
    {
        distances[i].index = i;

        OCTreeNode* child = node->childs[i];
        if (child != nullptr)
        {
            //countRayIntersect++;
            float near, far;
            if (child->bbox.rayIntersect(ray, near, far))
                distances[i].distance = near; // why near can be negative value???
            else
                distances[i].distance = FLT_MAX;
        }
        else
            distances[i].distance = FLT_MAX;
    }
    //std::sort(distances, distances + OCTREE_CHILD_COUNT, [&](OCTreeNodeDistance left, OCTreeNodeDistance right){
    //    return left.distance < right.distance;
    //});
    std::sort(distances, distances + OCTREE_CHILD_COUNT, ocTreeNodeDistanceSort);

    // 2. ray traversal
    for (int i = 0; i < OCTREE_CHILD_COUNT; i++)
    {
        OCTreeNodeDistance distance = distances[i];

        OCTreeNode* child = node->childs[distance.index];
        
        if (distance.distance > ray.maxt)
            return foundIntersection;
        
        if (!rayIntersectRecursivelyInOrder(mesh, child, ray, its, faceIndex, shadowRay))
            continue;
        foundIntersection = true;

        if (shadowRay)
            return true;
    }

    return foundIntersection;
}

bool AccelOCTree::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const
{
    //return Accel::rayIntersect(ray_, its, shadowRay);

    uint32_t faceIndex = (uint32_t)-1;
    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)
    bool intersected = false;
    if (traversal == OCTreeTraversal::EachNode)
        intersected = rayIntersectRecursivelyUnorder(m_mesh, rootNode, ray, its, faceIndex, shadowRay);
    else if (traversal == OCTreeTraversal::OrderedNodeByDistance)
        intersected = rayIntersectRecursivelyInOrder(m_mesh, rootNode, ray, its, faceIndex, shadowRay);

    if (intersected)
    {
        if (shadowRay)
            return true;

        calcIntersection(its, faceIndex);
        return true;
    }
    else
    {
        return false;
    }
}
NORI_NAMESPACE_END