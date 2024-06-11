/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 */

#ifndef MODULES_COMMON_MACRO_H_
#define MODULES_COMMON_MACRO_H_

#include <iostream>

#define DISALLOW_COPY_AND_ASSIGN(classname) \
 private:                                   \
  classname(const classname &);             \
  classname &operator=(const classname &);

///!构造函数在这里是私有的，所以不能直接创建对象
#define DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:                                         \
  classname();                                    \
  DISALLOW_COPY_AND_ASSIGN(classname);

#define DECLARE_SINGLETON(classname)        \
 public:                                    \
  static classname *instance() {            \
    static classname instance;              \
    return &instance;                       \
  }                                         \
  DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:
#endif  // MODULES_COMMON_MACRO_H_
/*
* 上面函数的用法定义了一个宏 DECLARE_SINGLETON，用于声明一个类的单例模式。
* 单例模式是一种设计模式，它保证一个类只有一个实例，并提供一个全局访问点。
* 宏调用了另一个宏 DISALLOW_IMPLICIT_CONSTRUCTORS，这个宏的作用是禁止类的默认构造函数、
* 拷贝构造函数和赋值操作符。这是为了防止用户创建类的其他实例，确保类的单例性。
* 这个宏以 private: 结束，这意味着在这个宏之后定义的成员都是私有的。
* 这是为了防止用户直接访问和修改类的内部状态，保证了类的封装性。
*/
