//-----------------------------------------------------------------------------
// Lonewolff
//
// Filename:    OgreText.h
// Description: Class for simple text in Ogre (Version 040507:18.30)
//-----------------------------------------------------------------------------

#include "OgreTextAreaOverlayElement.h"
#include "OgreStringConverter.h"
#include "OgreOverlayManager.h"
#include "OgreOverlayContainer.h"
#include "OgreOverlayElement.h"

using namespace Ogre;

#ifndef __OgreText_H__
#define __OgreText_H__

class OgreText
{
public:
    OgreText();
    ~OgreText();
    void setText(char *szString);
    void setText(String szString);
    void setPos(float x,float y);
    void setCol(float R,float G,float B,float I);
private:
    OverlayManager *olm;
    OverlayContainer *panel ;
    Overlay *overlay;
    TextAreaOverlayElement *text_area;
    static int init;
    int text_id;
    String sz_element;
};
#endif
