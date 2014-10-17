#include "overlay_utils.h"
#include <boost/lexical_cast.hpp>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreVector3.h>
#include "OGRE/OgreRoot.h"
#include "OGRE/OgreRenderSystem.h"
#include "OGRE/OgreRenderWindow.h"
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreEntity.h"
#include <OGRE/OgreSceneNode.h>
#include <ros/ros.h>

int OgreText::init=0;

OgreText::OgreText()
{
    text_id = this->init++;

    std::stringstream ss;
    ss << "NotificationRect" << text_id;

    const static uint32_t texture__data[1] = { 0x00000070 };
    Ogre::DataStreamPtr pixel_stream;
    pixel_stream.bind(new Ogre::MemoryDataStream( (void*)&texture__data[0], 4 ));

    texture_ = Ogre::TextureManager::getSingleton().loadRawData(ss.str() + "Texture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, 1, 1, Ogre::PF_R8G8B8A8, Ogre::TEX_TYPE_2D, 0);

    material_ = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_->setLightingEnabled(false);
    //material_->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->setCullingMode(Ogre::CULL_NONE);

    Ogre::TextureUnitState* tex_unit = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tex_unit->setTextureName(texture_->getName());
    tex_unit->setTextureFiltering( Ogre::TFO_NONE );

    olm = OverlayManager::getSingletonPtr();
    panel = static_cast<OverlayContainer*>(olm->createOverlayElement("Panel",std::string("GUI")+boost::lexical_cast<std::string>(text_id)));
    panel->setMetricsMode(Ogre::GMM_PIXELS);
    panel->setPosition(0, 0);
    panel->setDimensions(1, 1);
    panel->setMaterialName(material_->getName());

    overlay = olm->create(std::string("GUI_OVERLAY")+boost::lexical_cast<std::string>(text_id));
    overlay->add2D(panel);

    sz_element = "element_"+StringConverter::toString(init);
    text_area = static_cast<TextAreaOverlayElement*>(olm->createOverlayElement("TextArea",sz_element));
    text_area->setMetricsMode(Ogre::GMM_PIXELS);
    panel->addChild(text_area);
    overlay->show();

    ResourceGroupManager &resGroupMgr = ResourceGroupManager::getSingleton();
    if(!resGroupMgr.resourceGroupExists("Fonts"))
    {
       resGroupMgr.createResourceGroup("Fonts");
       // tell it to look at this location
       resGroupMgr.addResourceLocation("/usr/share/fonts/truetype/dejavu", "FileSystem");
       // get the font manager
       FontManager &fontMgr = FontManager::getSingleton();
       // create a font resource
       ResourcePtr font = fontMgr.create("DejaVuSansMono","Fonts");
    }


    // set as truetype
}

OgreText::~OgreText()
{
    sz_element="element_"+StringConverter::toString(text_id);
    olm->destroyOverlayElement(sz_element);
    olm->destroyOverlayElement("GUI");
    olm->destroy("GUI_OVERLAY");
    --(this->init);
}

void OgreText::setText(char *szString)
{
    text_area->setCaption(szString);
    text_area->setFontName("Arial");
    text_area->setCharHeight(18);

}

void OgreText::setText(String szString) // now You can use Ogre::String as text
{
    text_area->setCaption(szString);
    text_area->setFontName("Arial");
    text_area->setCharHeight(18);        
}

void OgreText::setPos(float x,float y)
{
    FontPtr font = Ogre::FontManager::getSingleton().getByName(text_area->getFontName());
    Real width,height;
    calculateTextPixelSize(text_area->getCaption(),font,text_area->getCharHeight(),width,height);
    //textArea->setPosition(x,y);
    panel->setPosition(x-width/2,y);
    panel->setDimensions(width,height);
}

void OgreText::setTextColor(float R,float G,float B,float I)
{
    text_area->setColour(Ogre::ColourValue(R,G,B,I));
}

void OgreText::calculateTextPixelSize(DisplayString text, FontPtr mpFont, Real mCharHeight, Real& width, Real& height)
{
    Real vpWidth, vpHeight;
    vpWidth = (Real) (OverlayManager::getSingleton().getViewportWidth());
    vpHeight = (Real) (OverlayManager::getSingleton().getViewportHeight());

    Real mViewportAspectCoef = vpHeight/vpWidth;

    height = mCharHeight;
    width = 0;

    Real len = 0.0f;
    for(DisplayString::iterator i = text.begin();i!=text.end();++i)
    {
        Font::CodePoint character = OGRE_DEREF_DISPLAYSTRING_ITERATOR(i);
        if (character == UNICODE_CR
                || character == UNICODE_NEL
                || character == UNICODE_LF)
        {
            height += mCharHeight;
            if(len > width)
                width = len;
            len = 0;
        }
        else if (character == UNICODE_SPACE) // space
        {
            len += mpFont->getGlyphAspectRatio(UNICODE_ZERO) * mCharHeight *1.5 * mViewportAspectCoef;
        }
        else
        {
            len += mpFont->getGlyphAspectRatio(character) * mCharHeight *1.5 * mViewportAspectCoef;
        }
    }
    if(len > width)
        width = len;
}

void OgreText::setPanelColor(int R, int G, int B, int I)
{
    // Get the pixel buffer
    HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();

    //directly modify pixel buffer in texture to change color

    // Lock the pixel buffer and get a pixel box
    pixelBuffer->lock(HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
    const PixelBox& pixelBox = pixelBuffer->getCurrentLock();

    uint8* pDest = static_cast<uint8*>(pixelBox.data);
    *pDest++ = R;
    *pDest++ = G;
    *pDest++ = B;
    *pDest++ = I;

    // Unlock the pixel buffer
    pixelBuffer->unlock();

    static int counter = 0;
    if(counter == 0)
    {
        Ogre::ResourceManager::ResourceMapIterator iterator = Ogre::FontManager::getSingleton().getResourceIterator();
        while(iterator.hasMoreElements())
        {
            Ogre::FontPtr font = iterator.getNext();
            ROS_ERROR("font : %s",font.get()->getName().c_str());
        }
        counter++;
    }


}
