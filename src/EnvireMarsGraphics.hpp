/**
 * \file EnvireMarsGraphics.hpp
 * \author Malte Langosz
 * \brief Plugin class to debug visualizations for physics representation based on envire items
 *
 */

#pragma once

#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/graph/TreeView.hpp>
#include <envire_core/events/GraphEventDispatcher.hpp>
#include <envire_core/events/GraphItemEventDispatcher.hpp>

#include <envire_types/Link.hpp>
#include <envire_types/Inertial.hpp>
#include <envire_types/geometry/Heightfield.hpp>
#include <envire_types/geometry/Plane.hpp>
#include <envire_types/geometry/Box.hpp>
#include <envire_types/geometry/Capsule.hpp>
#include <envire_types/geometry/Cylinder.hpp>
#include <envire_types/geometry/Mesh.hpp>
#include <envire_types/geometry/Sphere.hpp>
#include <envire_types/joints/Fixed.hpp>
#include <envire_types/joints/Revolute.hpp>
#include <envire_types/joints/Continuous.hpp>

#include <cfg_manager/CFGManagerInterface.h>
#include <data_broker/ProducerInterface.h>
#include <data_broker/DataBrokerInterface.h>
#include <data_broker/DataPackageMapping.h>

#include <mars_utils/Vector.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/AbsolutePose.hpp>


namespace mars
{
    namespace envire_mars_graphics
    {
        class EnvireMarsGraphics : public lib_manager::LibInterface,
                                   public interfaces::GraphicsUpdateInterface,
                                   public cfg_manager::CFGClient,
                                   public data_broker::ProducerInterface,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::Link>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::Inertial>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Heightfield>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Plane>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Box>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Capsule>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Cylinder>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Mesh>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::geometry::Sphere>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Fixed>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Revolute>>,
                                   public envire::core::GraphItemEventDispatcher<envire::core::Item<::envire::types::joints::Continuous>>
        {

        public:
            EnvireMarsGraphics(lib_manager::LibManager *theManager); ///< Constructor of the \c class Simulator.
            EnvireMarsGraphics(lib_manager::LibManager *theManager,
                               std::shared_ptr<envire::core::EnvireGraph> envireGraph,
                               std::shared_ptr<envire::core::TreeView> graphTreeView);
            virtual ~EnvireMarsGraphics();

            void init(void);

            // --- LibInterface ---
            int getLibVersion() const override
            {
                return 1;
            }

            const std::string getLibName() const override
            {
                return std::string{"envire_mars_graphics"};
            }

            CREATE_MODULE_INFO();

            // graphics callback
            virtual void preGraphicsUpdate(void) override;

            virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) override;

            // ## DataBroker callbacks ##
            virtual void produceData(const data_broker::DataInfo &info,
                                     data_broker::DataPackage *package,
                                     int callbackParam) override;

            // envire callbacks
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::Link>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::Inertial>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Heightfield>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Plane>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Box>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Capsule>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Cylinder>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Mesh>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::geometry::Sphere>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Fixed>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Revolute>>& e) override;
            virtual void itemAdded(const envire::core::TypedItemAddedEvent<envire::core::Item<::envire::types::joints::Continuous>>& e) override;

            void createVisual(configmaps::ConfigMap &node, envire::core::FrameId frameId);
            void createCollision(configmaps::ConfigMap &config, envire::core::FrameId frameId);
            void createJoint(const std::string &jointName, envire::core::FrameId frameId);

        private:
            std::shared_ptr<envire::core::EnvireGraph> envireGraph;
            std::shared_ptr<envire::core::TreeView> graphTreeView;

            // library references
            data_broker::DataBrokerInterface *dataBroker;
            interfaces::GraphicsManagerInterface *graphics;
            cfg_manager::CFGManagerInterface *cfg;

            data_broker::DataPackageMapping dbPackageMapping;
            std::map<unsigned long, interfaces::AbsolutePose*> visualMap, collisionMap, frameMap, anchorMap;
            //std::map<unsigned long, std::pair<envire::core::FrameId, Eigen::Affine3d>> visualAnchorMap;
            cfg_manager::cfgPropertyStruct cfgVisRep;
            bool showGui, showCollisions, showAnchor;
            double vizTime, avgVizTime, colTime, avgColTime, frameTime, avgFrameTime, anchorTime, avgAnchorTime;
            int avgTimeCount;
        };

    } // end of namespace envire_mars_graphics
} // end of namespace mars
