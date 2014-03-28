#ifndef FIELDVIEW_H
#define FIELDVIEW_H
#include <QGraphicsView>
class FieldView:public QGraphicsView
{
public:
  FieldView(QWidget* parent);
  void DrawField();
  void DrawBall(const std::vector<QPoint>& robotPositions);
  void DrawBlueRobots(const std::vector<QPoint>& robotPositions);
  void DrawRedRobots(const std::vector<QPoint>& robotPositions);
  void DrawWhiteRobots(const std::vector<QPoint>& robotPositions);
  void DrawBlackRobots(const std::vector<QPoint>& robotPositions);
  void DrawRef(const std::vector<QPoint>& robotPositions);
  QGraphicsScene Scene;
};

#endif // FIELDVIEW_H
