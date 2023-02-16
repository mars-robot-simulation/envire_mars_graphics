/**
 * \file EnvireMarsGraphics.hpp
 * \author Malte Langosz
 * \brief Plugin class to debug visualizations for physics representation based on envire items
 *
 */

#pragma once
#include <mars/utils/Vector.h>


#include <mars/cfg_manager/CFGManagerInterface.h>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>

#include <mars_ode_collision/objects/Object.hpp>

#include <iostream>

namespace mars
{
    namespace envire_mars_graphics
    {

        class EnvireMarsGraphics : public lib_manager::LibInterface,
                                    public interfaces::GraphicsUpdateInterface,
                                    public cfg_manager::CFGClient,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Frame>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Inertial>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Joint>>,
                                    public envire::core::GraphItemEventDispatcher<envire::core::Item<::smurf::Visual>>
        {

        public:
            EnvireMarsGraphics(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            virtual ~EnvireMarsGraphics();

            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string("envire_mars_graphics");
            }

            CREATE_MODULE_INFO();

            // graphics callback
            virtual void preGraphicsUpdate(void) override;

            virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) override;

            // envire callbacks
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Frame>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Inertial>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Joint>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::smurf::Visual>>& e) override;

        private:
            interfaces::GraphicsManagerInterface *graphics;
            cfg_manager::CFGManagerInterface *cfg;
            std::map<unsigned long, envire::core::FrameId> visualMap, visualFrameMap;
            std::map<unsigned long, std::pair<envire::core::FrameId, Eigen::Affine3d>> visualAnchorMap;
            cfg_manager::cfgPropertyStruct cfgVisRep;
        };

    } // end of namespace envire_ode_physics_viz
} // end of namespace mars
