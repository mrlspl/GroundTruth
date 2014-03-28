#ifndef MSHLABEL_H
#define MSHLABEL_H
#include <QLabel>
#include <QDebug>
#include <QMouseEvent>
#include <QGraphicsItem>
#include <QGraphicsView>
#include <vector>
typedef std::vector<QLineF>* ConstLineVector;
class MSHLabel : public QGraphicsView
{
Q_OBJECT

public:
MSHLabel(QGraphicsScene& Scene,QWidget* parent = 0 );
//void showImage(Image image);
public slots:
void showImage(unsigned char* image,int width,int height);
void showImage(const QImage&,ConstLineVector);
void showScene(const QGraphicsScene& scene);

signals:
void clicked(QMouseEvent*);
void onMove(QMouseEvent*);

protected:
void mousePressEvent ( QMouseEvent * event );
void mouseMoveEvent( QMouseEvent * event );
private:
 QGraphicsScene &Scene;
};
#endif // MSHLABEL_H
