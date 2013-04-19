/* 
 * RobotModel class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials and the .
 * 
 * Latest changes (12/08/2012):
 * - created class
 */

#ifndef MAIN_3D_VIEW_H
#define MAIN_3D_VIEW_H

#include <QWidget>

#include "base_3d_view.h"

namespace vigir_ocs
{
// Class "Main3DView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class Main3DView: public Base3DView
{
    Q_OBJECT
public:
    Main3DView( QWidget* parent = 0 );
    virtual ~Main3DView();
};
}
#endif // MAIN_3D_VIEW_H
