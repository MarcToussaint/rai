/*  ------------------------------------------------------------------
    Copyright (c) 2011-2020 Marc Toussaint
    email: toussaint@tu-berlin.de

    This code is distributed under the MIT License.
    Please see <root-path>/LICENSE for details.
    --------------------------------------------------------------  */

/****************************************************************************
** Meta object code from reading C++ file 'opengl_qt.h'
**
** Created: Tue Oct 16 12:50:33 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "opengl_qt.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'opengl_qt.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_sOpenGL[] = {

// content:
  6,       // revision
  0,       // classname
  0,    0, // classinfo
  0,    0, // methods
  0,    0, // properties
  0,    0, // enums/sets
  0,    0, // constructors
  0,       // flags
  0,       // signalCount

  0        // eod
};

static const char qt_meta_stringdata_sOpenGL[] = {
  "sOpenGL\0"
};

void sOpenGL::qt_static_metacall(QObject* _o, QMetaObject::Call _c, int _id, void** _a) {
  Q_UNUSED(_o);
  Q_UNUSED(_id);
  Q_UNUSED(_c);
  Q_UNUSED(_a);
}

const QMetaObjectExtraData sOpenGL::staticMetaObjectExtraData = {
  0,  qt_static_metacall
};

const QMetaObject sOpenGL::staticMetaObject = {
  {
    &QGLWidget::staticMetaObject, qt_meta_stringdata_sOpenGL,
    qt_meta_data_sOpenGL, &staticMetaObjectExtraData
  }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject& sOpenGL::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject* sOpenGL::metaObject() const {
  return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void* sOpenGL::qt_metacast(const char* _clname) {
  if(!_clname) return 0;
  if(!strcmp(_clname, qt_meta_stringdata_sOpenGL))
    return static_cast<void*>(const_cast< sOpenGL*>(this));
  return QGLWidget::qt_metacast(_clname);
}

int sOpenGL::qt_metacall(QMetaObject::Call _c, int _id, void** _a) {
  _id = QGLWidget::qt_metacall(_c, _id, _a);
  if(_id < 0)
    return _id;
  return _id;
}
QT_END_MOC_NAMESPACE
