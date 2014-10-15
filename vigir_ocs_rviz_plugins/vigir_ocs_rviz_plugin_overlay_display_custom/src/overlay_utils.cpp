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

int OgreText::init=0;

OgreText::OgreText()
{
    text_id = this->init++;

    std::stringstream ss;
    ss << "NotificationRect" << text_id;

    const static uint32_t texture_data[1] = { 0x00000070 };
    Ogre::DataStreamPtr pixel_stream;
    pixel_stream.bind(new Ogre::MemoryDataStream( (void*)&texture_data[0], 4 ));

    Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().loadRawData(ss.str() + "Texture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, pixel_stream, 1, 1, Ogre::PF_R8G8B8A8, Ogre::TEX_TYPE_2D, 0);

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setLightingEnabled(false);
    //material->getTechnique(0)->getPass(0)->setPolygonMode(Ogre::PM_WIREFRAME);
    material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->setCullingMode(Ogre::CULL_NONE);

    Ogre::TextureUnitState* tex_unit = material->getTechnique(0)->getPass(0)->createTextureUnitState();
    tex_unit->setTextureName(tex->getName());
    tex_unit->setTextureFiltering( Ogre::TFO_NONE );

    olm = OverlayManager::getSingletonPtr();
    panel = static_cast<OverlayContainer*>(olm->createOverlayElement("Panel",std::string("GUI")+boost::lexical_cast<std::string>(text_id)));
    panel->setMetricsMode(Ogre::GMM_PIXELS);
    panel->setPosition(0, 0);
    panel->setDimensions(200, 200);
    panel->setMaterialName(material->getName());

    overlay = olm->create(std::string("GUI_OVERLAY")+boost::lexical_cast<std::string>(text_id));
    overlay->add2D(panel);

    sz_element = "element_"+StringConverter::toString(init);
    text_area = static_cast<TextAreaOverlayElement*>(olm->createOverlayElement("TextArea",sz_element));
    text_area->setMetricsMode(Ogre::GMM_PIXELS);
    panel->addChild(text_area);
    overlay->show();
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
    text_area->setDimensions(1.0f,1.0f);
    text_area->setMetricsMode(Ogre::GMM_RELATIVE);
    text_area->setFontName("Arial");
    text_area->setCharHeight(0.03f);
}

void OgreText::setText(String szString) // now You can use Ogre::String as text
{
    text_area->setCaption(szString);
    text_area->setDimensions(1.0f,1.0f);
    text_area->setMetricsMode(Ogre::GMM_RELATIVE);
    text_area->setFontName("Arial");
    text_area->setCharHeight(0.03f);
}

void OgreText::setPos(float x,float y)
{
    //textArea->setPosition(x,y);
    panel->setPosition(x,y);
}

void OgreText::setCol(float R,float G,float B,float I)
{
    text_area->setColour(Ogre::ColourValue(R,G,B,I));
}
