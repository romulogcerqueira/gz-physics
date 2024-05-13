/*
 * Copyright (C) 2024 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GZ_PHYSICS_GETRAYINTERSECTIONS_HH_
#define GZ_PHYSICS_GETRAYINTERSECTIONS_HH_

#include <vector>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/Geometry.hh>
#include <gz/physics/SpecifyData.hh>

namespace gz
{
namespace physics
{
/// \brief GetRayIntersectionsFromLastStepFeature is a feature for retrieving the list
/// of ray intersections generated in the previous simulation step.
class GZ_PHYSICS_VISIBLE GetRayIntersectionsFromLastStepFeature
    : public virtual FeatureWithRequirements<ForwardStep>
{
  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using Scalar = typename PolicyT::Scalar;
    public: using ShapePtrType = ShapePtr<PolicyT, FeaturesT>;
    public: using VectorType = typename FromPolicy<PolicyT>::template Use<LinearVector>;

    struct RayIntersection
    {
      /// \brief The collision shape the ray hit
      ShapePtrType collision;

      /// \brief The hit point in the world coordinates
      VectorType point;

      /// \brief The fraction from "from" point to "to" point
      Scalar fraction;

      /// \brief The normal at the point in the world coordinates
      VectorType normal;
    };

    // public: using RayIntersection = RayIntersectionT<PolicyT>;
    public: using RayIntersectionData = SpecifyData<RequireData<RayIntersection>>;

    /// \brief Get intersections generated in the previous simulation step
    public: RayIntersectionData GetRayIntersectionsFromLastStep(
      const VectorType &_from, const VectorType &_to) const;
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    // public: using RayIntersection = RayIntersectionT<PolicyT>;
    public: using Scalar = typename PolicyT::Scalar;
    public: using VectorType = typename FromPolicy<PolicyT>::template Use<LinearVector>;

    public: struct RayIntersectionInternal
    {
      /// \brief Identity of the collision object
      // Identity collision;

      /// \brief The hit point in the world coordinates
      VectorType point;

      /// \brief The fraction from "from" point to "to" point
      Scalar fraction;

      /// \brief The normal at the point in the world coordinates
      VectorType normal;
    };

    public: virtual RayIntersectionInternal GetRayIntersectionsFromLastStep(
      const Identity &_worldID,
      const VectorType &_from,
      const VectorType &_to) const = 0;
  };
};
}
}

#include "gz/physics/detail/GetRayIntersections.hh"

#endif /* end of include guard: GZ_PHYSICS_GETRAYINTERSECTIONS_HH_ */
