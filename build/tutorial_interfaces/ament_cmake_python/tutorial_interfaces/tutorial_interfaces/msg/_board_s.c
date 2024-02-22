// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from tutorial_interfaces:msg/Board.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "tutorial_interfaces/msg/detail/board__struct.h"
#include "tutorial_interfaces/msg/detail/board__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool tutorial_interfaces__msg__board__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[37];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("tutorial_interfaces.msg._board.Board", full_classname_dest, 36) == 0);
  }
  tutorial_interfaces__msg__Board * ros_message = _ros_message;
  {  // key_up
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_up");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_up = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_down
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_down");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_down = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_left");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_left = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_right");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_right = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_left_shift
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_left_shift");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_left_shift = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // key_left_ctrl
    PyObject * field = PyObject_GetAttrString(_pymsg, "key_left_ctrl");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->key_left_ctrl = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // w
    PyObject * field = PyObject_GetAttrString(_pymsg, "w");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->w = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // a
    PyObject * field = PyObject_GetAttrString(_pymsg, "a");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->a = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // s
    PyObject * field = PyObject_GetAttrString(_pymsg, "s");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->s = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // d
    PyObject * field = PyObject_GetAttrString(_pymsg, "d");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->d = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // q
    PyObject * field = PyObject_GetAttrString(_pymsg, "q");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->q = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // e
    PyObject * field = PyObject_GetAttrString(_pymsg, "e");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->e = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // c
    PyObject * field = PyObject_GetAttrString(_pymsg, "c");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->c = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // x
    PyObject * field = PyObject_GetAttrString(_pymsg, "x");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->x = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // z
    PyObject * field = PyObject_GetAttrString(_pymsg, "z");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->z = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // j
    PyObject * field = PyObject_GetAttrString(_pymsg, "j");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->j = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * tutorial_interfaces__msg__board__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Board */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("tutorial_interfaces.msg._board");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Board");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  tutorial_interfaces__msg__Board * ros_message = (tutorial_interfaces__msg__Board *)raw_ros_message;
  {  // key_up
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_up);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_up", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_down
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_down);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_down", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_left
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_right
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_left_shift
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_left_shift);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_left_shift", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // key_left_ctrl
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->key_left_ctrl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "key_left_ctrl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // w
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->w);
    {
      int rc = PyObject_SetAttrString(_pymessage, "w", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // a
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->a);
    {
      int rc = PyObject_SetAttrString(_pymessage, "a", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // s
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->d);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // q
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->q);
    {
      int rc = PyObject_SetAttrString(_pymessage, "q", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // e
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->e);
    {
      int rc = PyObject_SetAttrString(_pymessage, "e", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // c
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->c);
    {
      int rc = PyObject_SetAttrString(_pymessage, "c", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // j
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->j);
    {
      int rc = PyObject_SetAttrString(_pymessage, "j", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
