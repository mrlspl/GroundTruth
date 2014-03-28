#include "MSHLabel.h"
#include <QMouseEvent>
#include <QImage>
#include <iostream>
#include <QCoreApplication>
#include "../Defines.h"
using namespace std;

MSHLabel::MSHLabel(QGraphicsScene& Scene,QWidget * parent)
    :QGraphicsView(parent),Scene(Scene)
{
    qRegisterMetaType<ConstLineVector>("ConstLineVector");
    setScene((QGraphicsScene*)&Scene);
    setResizeAnchor(QGraphicsView::AnchorUnderMouse); // TODO: this shall let you resize the picbox with scroll but
                                                        //does not work yet
}

void MSHLabel::mousePressEvent ( QMouseEvent* event )
{
  emit clicked(event);
}
void MSHLabel::mouseMoveEvent(QMouseEvent *event)
{
  emit onMove(event);
}

//      void MSHLabel::showImage(Image image)
//      {
//          Image rgbImg(image.Width,image.Height,CHANNEL_NUMBER_RGB,DEPTH_BITS,"RGB");
//          if(image.colorModel == Image::YUYV)//TODO:take care about the 'yuv' check
//              ColorModelConversions::fromYCbCrToRGB((unsigned char*)image.imageData,(unsigned char*)rgbImg.imageData,image.Width,image.Height);
//          else
//              rgbImg.imageData = image.imageData;

//          QImage rgbTMP(rgbImg.imageData,
//                        rgbImg.Width,
//                        rgbImg.Height,
//                        QImage::Format_RGB888);

//          this->setPixmap(QPixmap::fromImage(rgbTMP));
//          this->adjustSize();
//      }
void MSHLabel::showImage(unsigned char* image,int width,int height)
{

  width = WIDTH;
  height = HEIGHT;
  QImage rgbTMP((unsigned char*)image,
                width,
                height,
                QImage::Format_RGB888);
  const QPixmap pix = QPixmap::fromImage(rgbTMP);

//  QPainter painter(&pix);
  Scene.addPixmap(pix);
//  painter.setRenderHint(QPainter::Antialiasing);
//  scene.render(&painter);

//  setScene(&scene);|
  adjustSize();
  QCoreApplication::processEvents();
//  painter.end();
}

void MSHLabel::showScene(const QGraphicsScene&  scene)
{
//  setScene((QGraphicsScene*)&scene);
    adjustSize();
  QCoreApplication::processEvents();
}
void MSHLabel::showImage(const QImage& image,ConstLineVector items)
{
    const QPixmap pix = QPixmap::fromImage(image);
    Scene.addPixmap(pix);
    for(int i=0;i<items->size();i++)
        Scene.addLine((*items)[i]);
    adjustSize();
    QCoreApplication::processEvents();
}
