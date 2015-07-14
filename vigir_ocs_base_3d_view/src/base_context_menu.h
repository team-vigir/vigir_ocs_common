/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
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
