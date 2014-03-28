#include "fieldview.h"
#include "vector"
#include "Defines.h"
#include "iostream"
FieldView::FieldView(QWidget* parent=0):QGraphicsView(parent)
{
    setScene(&Scene);

}
void FieldView::DrawField()
{
  Scene.setBackgroundBrush(QBrush(Qt::green));
  Scene.addRect(FIELD_BORDER,FIELD_BORDER,FIELD_WIDTH,FIELD_HEIGHT,QPen(QBrush(Qt::white),LINE_WIDTH_PIX));
  Scene.addLine(FIELD_BORDER+FIELD_WIDTH/2,FIELD_BORDER,FIELD_BORDER+FIELD_WIDTH/2,FIELD_BORDER+FIELD_HEIGHT,QPen(Qt::white,LINE_WIDTH_PIX));
  Scene.addEllipse(CENTER_X - CIRCLE_RADIUS_PIX,CENTER_Y - CIRCLE_RADIUS_PIX,CIRCLE_RADIUS_PIX*2,CIRCLE_RADIUS_PIX*2,QPen(QBrush(Qt::white),LINE_WIDTH_PIX));
}

void FieldView::DrawBall(const std::vector<QPoint>& robotPositions)
{
  for(int i=0;i<robotPositions.size();i++)
  {
    Scene.addEllipse(robotPositions[i].x() - BALL_SIZE_PIX/2, robotPositions[i].y() - BALL_SIZE_PIX/2,BALL_SIZE_PIX,BALL_SIZE_PIX,QPen(QBrush(Qt::yellow),BALL_WIDTH_PIX));

  }
  update();
}
void FieldView::DrawBlueRobots(const std::vector<QPoint>& robotPositions)
{
  for(int i=0;i<robotPositions.size();i++)
  {
    Scene.addEllipse(robotPositions[i].x() - ROBOT_SIZE_PIX/2, robotPositions[i].y() - ROBOT_SIZE_PIX/2,ROBOT_SIZE_PIX,ROBOT_SIZE_PIX,QPen(QBrush(Qt::blue),ROBOT_WIDTH_PIX));
//    std::cout << " X : " << robotPositions[i].x()  << " Y : " << robotPositions[i].y() <<std::endl;
  }
  update();
}
void FieldView::DrawRedRobots(const std::vector<QPoint>& robotPositions)
{
  for(int i=0;i<robotPositions.size();i++)
  {
    Scene.addEllipse(robotPositions[i].x() - ROBOT_SIZE_PIX/2, robotPositions[i].y() - ROBOT_SIZE_PIX/2,ROBOT_SIZE_PIX,ROBOT_SIZE_PIX,QPen(QBrush(Qt::red),ROBOT_WIDTH_PIX));
//    std::cout << " X : " << robotPositions[i].x()  << " Y : " << robotPositions[i].y() <<std::endl;
  }
  update();
}
void FieldView::DrawBlackRobots(const std::vector<QPoint>& robotPositions)
{
  for(int i=0;i<robotPositions.size();i++)
  {
    Scene.addEllipse(robotPositions[i].x() - ROBOT_SIZE_PIX/2, robotPositions[i].y() - ROBOT_SIZE_PIX/2,ROBOT_SIZE_PIX,ROBOT_SIZE_PIX,QPen(QBrush(Qt::white),ROBOT_WIDTH_PIX));
//    std::cout << " X : " << robotPositions[i].x()  << " Y : " << robotPositions[i].y() <<std::endl;
  }
  update();
}
void FieldView::DrawWhiteRobots(const std::vector<QPoint>& robotPositions)
{
  for(int i=0;i<robotPositions.size();i++)
  {
    Scene.addEllipse(robotPositions[i].x() - ROBOT_SIZE_PIX/2, robotPositions[i].y() - ROBOT_SIZE_PIX/2,ROBOT_SIZE_PIX,ROBOT_SIZE_PIX,QPen(QBrush(Qt::white),ROBOT_WIDTH_PIX));
//    std::cout << " X : " << robotPositions[i].x()  << " Y : " << robotPositions[i].y() <<std::endl;
  }
  update();
}
void FieldView::DrawRef(const std::vector<QPoint>& robotPositions)
{
  for(int i=0;i<robotPositions.size();i++)
  {
    Scene.addEllipse(robotPositions[i].x() - ROBOT_SIZE_PIX/2, robotPositions[i].y() - ROBOT_SIZE_PIX/2,ROBOT_SIZE_PIX,ROBOT_SIZE_PIX,QPen(QBrush(Qt::black),ROBOT_WIDTH_PIX));
//    std::cout << " X : " << robotPositions[i].x()  << " Y : " << robotPositions[i].y() <<std::endl;
  }
  update();
}
