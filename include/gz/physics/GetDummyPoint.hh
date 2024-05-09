/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef GZ_PHYSICS_GETDUMMYPOINT_HH_
#define GZ_PHYSICS_GETDUMMYPOINT_HH_

#include <vector>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/Geometry.hh>
#include <gz/physics/SpecifyData.hh>

namespace gz
{
namespace physics
{
/// \brief GetDummyPointFromLastStepFeature is a feature for retrieving the list
/// of contacts generated in the previous simulation step.
class GZ_PHYSICS_VISIBLE GetDummyPointFromLastStepFeature
    : public virtual FeatureWithRequirements<ForwardStep>
{
  public: template <typename PolicyT>
  struct DummyPointT
  {
    /// \brief The point of contact expressed in the world frame
    gz::math::Vector3d point;
  };

  public: template <typename PolicyT, typename FeaturesT>
  class World : public virtual Feature::World<PolicyT, FeaturesT>
  {
    public: using DummyPoint = DummyPointT<PolicyT>;
    public: using Dummy = SpecifyData<RequireData<DummyPoint>>;

    /// \brief Get contacts generated in the previous simulation step
    public: Dummy GetDummyPointFromLastStep(
      const gz::math::Vector3d &_from, const gz::math::Vector3d &_end
    ) const;
  };

  public: template <typename PolicyT>
  class Implementation : public virtual Feature::Implementation<PolicyT>
  {
    public: using DummyPoint = DummyPointT<PolicyT>;

    public: virtual DummyPoint GetDummyPointFromLastStep(
        const Identity &_worldID,
        const gz::math::Vector3d &_from,
        const gz::math::Vector3d &_to) const = 0;
  };
};
}
}

#include "gz/physics/detail/GetDummyPoint.hh"

#endif /* end of include guard: GZ_PHYSICS_GETDUMMYPOINT_HH_ */
