
#ifndef BASE_CONTEXT_MENU_H
#define BASE_CONTEXT_MENU_H

#include <QObject>

#include <ros/ros.h>

#include <string>
#include <boost/bind.hpp>
#include <vector>
#include <map>
#include <stdlib.h>
#include <QTreeWidget>
#include "context_menu_manager.h"
#include "footstep_vis_manager.h"
#include "base_3d_view.h"


namespace vigir_ocs
{
class BaseContextMenu: public ContextMenuManager
{
    Q_OBJECT

    //friend to access certain protected methods for context callbacks
    friend class Base3DView;

public:
    BaseContextMenu(Base3DView* base_3d_view, FootstepVisManager* footstep_vis_manager);
    virtual ~BaseContextMenu();

    void setTemplateTree(QTreeWidget * root);
    void resetMenu();
    //QMenu *getContextMenu();
    void addToContextMenuExternally(std::vector<contextMenuItem* > external_context_menu);
    void processContextMenuVector(QAction* context_menu_selected_item);
    void setItemVisibility(QString name, bool visibility);
    void setActiveContext(std::string name);

Q_SIGNALS:


private:
   //process Vector to build menu + actions correctly, base3dview is the only one that builds, others like MainView must send their context items to base
   void buildContextMenuHeirarchy();
   void createContextMenuItems();
   void addTemplatesToContext();
   void createContextMenu(bool, int x, int y);
   void processContextMenu(int x, int y);

   //CALLBACKS///////////////////////////////////
    void selectTemplate();


   //END CALLBACKS///////////////////////////

   //Only Base3dView constructs and interacts with the context menu
   QMenu* context_menu_;
   QAction* context_menu_selected_item_;
   int initializing_context_menu_;

   FootstepVisManager* footstep_vis_manager_;
   Base3DView* base_3d_view_;
   QTreeWidget * template_root_;
   std::string active_context_name_;


   //Menu Items/////////////////////////
   contextMenuItem * selectFootstepGoalMenu;
   contextMenuItem * insertTemplateMenu;
   contextMenuItem * removeTemplateMenu;
   contextMenuItem * selectTemplateMenu;
   contextMenuItem * removeFootstepMenu;
   contextMenuItem * selectFootstepMenu;
   contextMenuItem * selectStartFootstepMenu;
   contextMenuItem * clearStartFootstepMenu;
   contextMenuItem * lockFootstepMenu;
   contextMenuItem * unlockFootstepMenu;
   contextMenuItem * undoFootstepMenu;
   contextMenuItem * redoFootstepMenu;
   contextMenuItem * footstepGoalMenu;
   contextMenuItem * stitchFootstepMenu;
   contextMenuItem * defaultFootstepRequestMenu;
   contextMenuItem * customFootstepRequestMenu;
   contextMenuItem * executeFootstepPlanMenu;
   contextMenuItem * cartesianMotionMenu;
   contextMenuItem * createCartesianMarkerMenu;
   contextMenuItem * removeCartesianMarkerMenu;
   contextMenuItem * circularMotionMenu;
   contextMenuItem * createCircularMarkerMenu;
   contextMenuItem * removeCircularMarkerMenu;
   contextMenuItem * lockLeftMenu;
   contextMenuItem * lockRightMenu;
   contextMenuItem * unlockArmsMenu;
   contextMenuItem * snapHandMenu;
   contextMenuItem * leftArmMenu;
   contextMenuItem * rightArmMenu;

};

}
#endif //BASE_CONTEXT_MENU_H
