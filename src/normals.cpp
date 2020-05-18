#include <nori/integrator.h>
#include <nori/scene.h>

using namespace std;

NORI_NAMESPACE_BEGIN

class NormalIntegrator : public Integrator
{
public:
    NormalIntegrator(const PropertyList &props)
    {
        /* Find the  */
    }

    /// Compute the radiance value for a given ray. Just return green here
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        /* Return the component-wise absolute value of the shading normal as a color */
        Normal3f n = its.shFrame.n.cwiseAbs();
        return Color3f(n.x(), n.y(), n.z());
    }

    std::string toString() const {
        return tfm::format("NormalIntegrator[]");
    }

protected:
    std::string m_myProperty;
};

NORI_REGISTER_CLASS(NormalIntegrator, "normals");
NORI_NAMESPACE_END