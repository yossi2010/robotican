#ifndef EVENTSIGNAL_H
#define EVENTSIGNAL_H
#include <QObject>
#include <QMainWindow>
#include <QApplication>
#include <QLabel>
#include "led.h"

class EventSignal : public QObject {

    Q_OBJECT
public:
    EventSignal();
    void signalBatVal(int newVal);
    void signalLed(long int newVal, Led* led);

    Q_SIGNALS:
    void batValChanged(int newVal);
    void ledChanged(long int newVal, Led* led);

private:
    int _batPwrVal;
};

#endif //EVENTSIGNAL_H
