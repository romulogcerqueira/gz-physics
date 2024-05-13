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

#ifndef GZ_PHYSICS_DETAIL_GETRAYINTERSECTIONS_HH_
#define GZ_PHYSICS_DETAIL_GETRAYINTERSECTIONS_HH_

#include <utility>
#include <vector>
#include <gz/physics/GetRayIntersections.hh>

namespace gz
{
namespace physics
{
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto GetRayIntersectionsFromLastStepFeature::World<
    PolicyT, FeaturesT>::GetRayIntersectionsFromLastStep(
      const VectorType &_from, const VectorType &_to
    ) const -> RayIntersectionData
{
  auto result =
      this->template Interface<GetRayIntersectionsFromLastStepFeature>()
          ->GetRayIntersectionsFromLastStep(this->identity, _from, _to);

  RayIntersection intersection{
      ShapePtrType(this->pimpl, result.collision),
      result.point,
      result.fraction,
      result.normal
  };

  RayIntersectionData output;
  output.template Get<RayIntersection>() = std::move(intersection);
  return output;
}

}  // namespace physics
}  // namespace gz

#endif
