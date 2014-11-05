
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

class BaseContextMenu: public QObject
{
    Q_OBJECT



public:
    BaseContextMenu(Base3DView* base_3d_view, FootstepVisManager* footstep_vis_manager);
    virtual ~BaseContextMenu();

    void setTemplateTree(QTreeWidget * root);
    void resetMenu();
    //QMenu *getContextMenu();
    //void addToContextMenuExternally(std::vector<contextMenuItem* > external_context_menu);

    void setItemVisibility(QString name, bool visibility);
    void setActiveContext(std::string name, int num);


private:  
   void createContextMenuItems();
   void addTemplatesToContext();  

   //CALLBACKS///////////////////////////////////
   // void selectTemplate();
    void contextInsertTemplate(QString path);


   //END CALLBACKS///////////////////////////



   FootstepVisManager* footstep_vis_manager_;
   Base3DView* base_3d_view_;
   QTreeWidget * template_root_;

   //need to build template tree
   contextMenuItem * insertTemplateMenu;

   std::vector<contextMenuItem*> context_menu_items_;

   //Menu Items/////////////////////////
//   contextMenuItem * selectFootstepGoalMenu;

//   contextMenuItem * removeTemplateMenu;
//   contextMenuItem * selectTemplateMenu;
//   contextMenuItem * removeFootstepMenu;
//   contextMenuItem * selectFootstepMenu;
//   contextMenuItem * selectStartFootstepMenu;
//   contextMenuItem * clearStartFootstepMenu;
//   contextMenuItem * lockFootstepMenu;
//   contextMenuItem * unlockFootstepMenu;
//   contextMenuItem * undoFootstepMenu;
//   contextMenuItem * redoFootstepMenu;
//   contextMenuItem * footstepGoalMenu;
//   contextMenuItem * stitchFootstepMenu;
//   contextMenuItem * defaultFootstepRequestMenu;
//   contextMenuItem * customFootstepRequestMenu;
//   contextMenuItem * executeFootstepPlanMenu;
//   contextMenuItem * cartesianMotionMenu;
//   contextMenuItem * createCartesianMarkerMenu;
//   contextMenuItem * removeCartesianMarkerMenu;
//   contextMenuItem * circularMotionMenu;
//   contextMenuItem * createCircularMarkerMenu;
//   contextMenuItem * removeCircularMarkerMenu;
//   contextMenuItem * lockLeftMenu;
//   contextMenuItem * lockRightMenu;
//   contextMenuItem * unlockArmsMenu;
//   contextMenuItem * snapHandMenu;
//   contextMenuItem * leftArmMenu;
//   contextMenuItem * rightArmMenu;

};

}
#endif //BASE_CONTEXT_MENU_H
