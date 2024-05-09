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

#ifndef GZ_PHYSICS_DETAIL_GETDUMMYPOINT_HH_
#define GZ_PHYSICS_DETAIL_GETDUMMYPOINT_HH_

#include <utility>
#include <vector>
#include <gz/physics/GetDummyPoint.hh>

namespace gz
{
namespace physics
{
/////////////////////////////////////////////////
template <typename PolicyT, typename FeaturesT>
auto GetDummyPointFromLastStepFeature::World<
    PolicyT, FeaturesT>::GetDummyPointFromLastStep(
      const VectorType &_from, const VectorType &_end
    ) const -> Dummy
{
  auto dummyPointInternal =
      this->template Interface<GetDummyPointFromLastStepFeature>()
          ->GetDummyPointFromLastStep(this->identity, _from, _end);

  DummyPoint point {dummyPointInternal.point};
  Dummy output;
  output.template Get<DummyPoint>() = std::move(point);
  return output;
}

}  // namespace physics
}  // namespace gz

#endif
