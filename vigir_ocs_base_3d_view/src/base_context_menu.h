
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
#include "base_3d_view.h"
#include "footstep_vis_manager.h"


namespace vigir_ocs
{
class BaseContextMenu: public ContextMenuManager
{
    Q_OBJECT
public:
    BaseContextMenu(Base3DView* base_3d_view, FootstepVisManager* footstep_vis_manager);
    virtual ~BaseContextMenu();

    void setTemplateTree(QTreeWidget * root);
    void getContextMenu();
    void addToContextMenuExternally(std::vector<contextMenuItem* > external_context_menu);
    void processContextMenuVector();
Q_SIGNALS:


private:
   //process Vector to build menu + actions correctly, base3dview is the only one that builds, others like MainView must send their context items to base
   void buildContextMenu();
   void addTemplatesToContext();




   //CALLBACKS///////////////////////////////////



   //END CALLBACKS///////////////////////////


   FootstepVisManager* footstep_vis_manager_;
   Base3DView* base_3d_view_;

   QTreeWidget * template_root_;
   bool context_menu_created_;

};

}
#endif BASE_CONTEXT_MENU_H
