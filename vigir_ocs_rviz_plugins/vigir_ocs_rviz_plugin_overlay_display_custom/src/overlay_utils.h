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
#include "OgreHardwarePixelBuffer.h"
#include <OgreFontManager.h>

using namespace Ogre;

#ifndef __OgreText_H__
#define __OgreText_H__

#define UNICODE_NEL 0x0085
 #define UNICODE_CR 0x000D
 #define UNICODE_LF 0x000A
 #define UNICODE_SPACE 0x0020
 #define UNICODE_ZERO 0x0030

class OgreText
{
public:
    OgreText();
    ~OgreText();
    void setText(char *szString);
    void setText(String szString);
    void setPos(float x,float y);
    void setTextColor(float R,float G,float B,float I);
    void setPanelColor(int R, int G, int B, int I);
    void calculateTextPixelSize(DisplayString text, FontPtr mpFont, Real mCharHeight, Real& width, Real& height);
private:
    OverlayManager *olm;
    OverlayContainer *panel ;
    Overlay *overlay;
    TextAreaOverlayElement *text_area;
    static int init;
    int text_id;
    String sz_element;
    Ogre::MaterialPtr material_;
    Ogre::TexturePtr texture_;
};
#endif
