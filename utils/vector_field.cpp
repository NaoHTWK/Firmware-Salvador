#include "vector_field.h"

#include "algorithm_ext.h"
#include "stl_ext.h"

namespace htwk::vectorfield {

point_2d calcDirection(const Position& source, const Position& target,
                       const std::vector<Influencer>& influencer) {

    // Lambda to find if an influencer is behind or in front of the direct path.
    // Basically the coefficient of the orthogonal projection is calcualted.
    // The projection is the source onto the influencer-target line. If the
    // coefficient is less then 0 the influencer is between source and target.
    auto coefficent = [&source, &target](const point_2d& p) {
        auto s = p - source.point();
        auto o = p - target.point();
        if (o.norm_sqr() < std::numeric_limits<float>::epsilon())
            return 1.f;
        return s.dot(o) / o.dot(o);
    };

    // Filter for relevant influencers in sphere of influence
    std::vector<Influencer> in_soi;
    insert_if(influencer, in_soi, [&source, &coefficent](const Influencer& infl) {
        // Check if we are in range of the influencer and if the obstacle is in front of us.
        return (source.point() - infl.position).norm_sqr() <= infl.radius * infl.radius &&
               coefficent(infl.position) <= 0.f;
    });

    // Shape the field around the influencer ...
    // For each influencer, we calculate the normal to the vector from the source to the influencer.
    // But because we don't want to run around circles, we have to differentiate between left and
    // right side of the influencer to target line.
    auto t_p = target.point();
    auto s_p = source.point();
    point_2d sum = accumulate(
            in_soi, point_2d{.0f, .0f}, [&s_p, &t_p](const point_2d& acc, const Influencer& infl) {
                auto internal_accumulator = acc;
                auto delta = (infl.position - s_p);

                // if we want to move away from the Influencer
                if (infl.flee) {
                    auto fleeting_vec = delta.normalized() * std::abs(infl.deflection);
                    internal_accumulator -= (infl.flee ? fleeting_vec : point_2d{0.f, 0.f});
                }

                // if we are not on top of the Influencer (distance 0)
                if (delta.norm_sqr() > std::numeric_limits<float>::epsilon()) {
                    auto deflection_strength = infl.deflection / delta.norm_sqr();
                    auto side = (s_p.x - infl.position.x) * (t_p.y - infl.position.x) -
                                (s_p.y - infl.position.y) * (t_p.x - infl.position.x);
                    internal_accumulator -=
                            sgn(side) * delta.normal().normalized() * deflection_strength;
                }
                return internal_accumulator;
            });

    return sum;
}

}  // namespace htwk::vectorfield
