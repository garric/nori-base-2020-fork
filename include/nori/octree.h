#include <nori/accel.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

/// forward declaration
class OCTreeNode;

class AccelOCTree : public Accel {
public:
    ~AccelOCTree();

    void build();

    bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const;

private:
    /// must be pointer; 
    /// or the compiler can't evaluate the size of OCTreeNode and can't allocate memory for this
    OCTreeNode* rootNode;
};

NORI_NAMESPACE_END