#ifndef BASE_CONTEXT_MENU_H
#define BASE_CONTEXT_MENU_H

#include <QObject>
#include <ros/ros.h>
#include <string>
#include <boost/bind.hpp>
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
    BaseContextMenu(Base3DView* base_3d_view);
    virtual ~BaseContextMenu();

    void setTemplateTree(QTreeWidget * root);
    void resetMenu();        
    void setItemVisibility(QString name, bool visibility);
    void setActiveContext(std::string name, int num);

private:  
   void createContextMenuItems();
   void addTemplatesToContext();  

   //CALLBACKS///////////////////////////////////   
    void contextInsertTemplate(QString path);
    void removeTemplateContextMenu();


   //END CALLBACKS//////////////////////////

   ContextMenuManager* context_menu_manager_;
   FootstepVisManager* footstep_vis_manager_;
   Base3DView* base_3d_view_;
   QTreeWidget * template_root_;

   //need to build template tree
   contextMenuItem * insertTemplateMenu_;
};

}
#endif //BASE_CONTEXT_MENU_H
