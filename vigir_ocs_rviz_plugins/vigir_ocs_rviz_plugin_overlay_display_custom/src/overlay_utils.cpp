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
@TODO_ADD_AUTHOR_INFO
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

OgreText::OgreText() :
    padding_(5.0)
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

    overlay_manager_ = OverlayManager::getSingletonPtr();
    panel_ = static_cast<OverlayContainer*>(overlay_manager_->createOverlayElement("Panel",std::string("GUI")+boost::lexical_cast<std::string>(text_id)));
    panel_->setMetricsMode(Ogre::GMM_PIXELS);
    panel_->setPosition(0, 0);
    panel_->setDimensions(1, 1);
    panel_->setMaterialName(material_->getName());

    overlay = overlay_manager_->create(std::string("GUI_OVERLAY")+boost::lexical_cast<std::string>(text_id));
    overlay->add2D(panel_);   

    sz_element = "element_"+StringConverter::toString(init);
    text_area_ = static_cast<TextAreaOverlayElement*>(overlay_manager_->createOverlayElement("TextArea",sz_element));
    text_area_->setMetricsMode(Ogre::GMM_PIXELS);
    panel_->addChild(text_area_);
    overlay->show();
}

OgreText::~OgreText()
{
    sz_element="element_"+StringConverter::toString(text_id);
    overlay_manager_->destroyOverlayElement(sz_element);
    overlay_manager_->destroyOverlayElement("GUI");
    overlay_manager_->destroy("GUI_OVERLAY");
    --(this->init);
}

void OgreText::setText(char *szString)
{
    text_area_->setCaption(szString);
    text_area_->setFontName("Arial");
    text_area_->setCharHeight(18);

}

void OgreText::setText(String szString) // now You can use Ogre::String as text
{
    text_area_->setCaption(szString);
    text_area_->setFontName("Arial");
    text_area_->setCharHeight(18);
}

void OgreText::setPos(float x,float y)
{
    FontPtr font = Ogre::FontManager::getSingleton().getByName(text_area_->getFontName());
    Real width,height;

    calculateTextPixelSize(text_area_->getCaption(),font,text_area_->getCharHeight(),width,height);
    text_area_->setPosition(padding_,padding_);
    panel_->setPosition(x-width/2-padding_,y-padding_);
    panel_->setDimensions(width+padding_*2,height+padding_);

}

void OgreText::setTextColor(float R,float G,float B,float I)
{
    text_area_->setColour(Ogre::ColourValue(R,G,B,I));
}

void OgreText::calculateTextPixelSize(DisplayString text, FontPtr mpFont, Real mCharHeight, Real& width, Real& height)
{
    Real vpWidth, vpHeight;
    vpWidth = (Real) (OverlayManager::getSingleton().getViewportWidth());
    vpHeight = (Real) (OverlayManager::getSingleton().getViewportHeight());
    //ROS_ERROR("[viewport] w: %f h: %f",vpWidth,vpHeight);   

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
            len += mpFont->getGlyphAspectRatio(UNICODE_ZERO) * mCharHeight;// * 2.0 * mViewportAspectCoef;
        }
        else
        {
            len += mpFont->getGlyphAspectRatio(character) * mCharHeight;// * 2.0 * mViewportAspectCoef;
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
}
