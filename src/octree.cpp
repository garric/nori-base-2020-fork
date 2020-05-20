#include <nori/octree.h>
#include <Eigen/Geometry>
#include <nori/common.h>
#include <vector>
#include <list>

NORI_NAMESPACE_BEGIN

const int OCTREE_CHILD_COUNT = 8;
const int OCTREE_LEAF_TRIANGLES = 10;
const float OCTREE_LEAF_MINBOUNDWIDTH = 1.0f / 32.0f;// 2.0f;

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

struct Triangle
{
public:
    bool overlaps(BoundingBox3f& bbox)
    {
        tempBoundingBox3f.set(p0, p0);
        tempBoundingBox3f.expandBy(p1);
        tempBoundingBox3f.expandBy(p2);
        
        return bbox.overlaps(tempBoundingBox3f);
    }

public:
    Point3f p0;
    Point3f p1;
    Point3f p2;

    uint32_t index;

private:
    static BoundingBox3f tempBoundingBox3f;
};

BoundingBox3f Triangle::tempBoundingBox3f;

class OCTreeNode
{
public:
    OCTreeNode()
    {
        parent = nullptr;
        for (int i = 0; i < OCTREE_CHILD_COUNT; i++)
        {
            childs[i] = nullptr;
        }
        bbox.reset();
    }

    ~OCTreeNode()
    {
        //delete[] childs; // invoke child's destructor
    }

public:
    OCTreeNode* parent;
    OCTreeNode* childs[OCTREE_CHILD_COUNT];

    BoundingBox3f bbox;

    std::vector<Triangle*>* triangles;
    
};

class OCTreeNodeLeaf : public OCTreeNode
{
public:
    std::vector<Triangle*>* triangles;
};

float_t distanceInfinite = 0.25f;// 1 << 16; // FLT_MAX / 2 - 100;
BoundingBox3f bboxInfinite(Point3f(-distanceInfinite, -distanceInfinite, -distanceInfinite), Point3f(distanceInfinite, distanceInfinite, distanceInfinite));
BoundingBox3f bboxTemp = bboxInfinite;

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

OCTreeNode* buildRecursively(BoundingBox3f& bbox, std::vector<Triangle*>* triangles)
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
            if (triangle->overlaps(bboxChild))
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
        OCTreeNode* child = buildRecursively(bboxChild, &splitedTriangles[i]);
        if (child == nullptr)
            continue;

        child->parent = node;
        node->childs[i] = child;

        child->bbox = bboxChild;
    }
    return node;
}

AccelOCTree::~AccelOCTree()
{
    delete rootNode;
}

void AccelOCTree::build()
{
    Accel::build();
    // build an octree

    std::vector<Triangle*> triangles;
    //std::vector<Triangle> triangles;
    for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
        Triangle* triangle = new Triangle();
        m_mesh->getTriangleVertices(idx, triangle->p0, triangle->p1, triangle->p2);
        triangle->index = idx;

        triangles.push_back(triangle);
    }
    //Point3f extents = m_mesh->getBoundingBox().getExtents();
    BoundingBox3f bbox = bboxInfinite; // default is infinite Bounding Box
    rootNode = buildRecursively(bbox, &triangles);
    rootNode->bbox = bbox;
    int a = 0;
}

bool AccelOCTree::rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const
{
    return Accel::rayIntersect(ray, its, shadowRay);
}
NORI_NAMESPACE_END