
/***************************************************************************
 *  skiller.cpp - Skiller main application
 *
 *  Created: Tue Aug  3 12:16:10 2010
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <ros/ros.h>

#include <lua_utils/context.h>

static int lua_add_watchfile(lua_State *L);

class SkillerMain
{
  friend int lua_add_watchfile(lua_State *L);
 public:
  SkillerMain(ros::NodeHandle &n)
    : __lua(/* watch files */ true, /* tracebacks */ true),
      __n(n)
  {
  }

  ~SkillerMain()
  {
  }

  void
  init_lua()
  {
    std::string skill_space = "herb_skills";
    if (__n.hasParam("/skiller/skillspace")) {
      __n.getParam("/skiller/skillspace", skill_space);
    }
    __lua.set_string("SKILLSPACE", skill_space.c_str());
    __lua.set_string("ROS_MASTER_URI", ros::master::getURI().c_str());

    __lua.add_package_dir(LUADIR);
    __lua.add_package("roslua");
    __lua.set_cfunction("add_watchfile", lua_add_watchfile);

    __lua.set_start_script(LUADIR"/skiller/ros/start.lua");
  }

  int run()
  {
    init_lua();

    ros::Rate rate(25);
    bool quit = false;
    // run until skiller stopped
    while (! quit && __n.ok() ) {
      __lua.get_global("roslua");	// roslua
      try {
	// Spin!
	__lua.get_field(-1, "spin");	// roslua roslua.spin
	try {
	  __lua.pcall();		// roslua
	} catch (Exception &e) {
	  printf("%s", e.what());
	}

	// get quite flag
	__lua.get_field(-1, "quit");	// roslua roslua.quit
	quit = __lua.to_boolean(-1);
	__lua.pop(2);			// ---
      } catch (Exception &e) {
	printf("%s\n", e.what());
      }
      rate.sleep();
    }
    __lua.get_global("roslua");		// roslua
    __lua.get_field(-1, "finalize");	// roslua roslua.finalize
    __lua.pcall();			// roslua
    __lua.pop(1);			// ---
    return 0;
  }

 private:
  fawkes::LuaContext __lua;
  ros::NodeHandle &__n;
};

static SkillerMain *skiller;

int
lua_add_watchfile(lua_State *L)
{
  const char *s = luaL_checkstring(L, 1);
  if (s == NULL) luaL_error(L, "Directory argument missing");
  try {
    skiller->__lua.add_watchfile(s);
  } catch (Exception &e) {
    luaL_error(L, "Adding watch directory failed: %s", e.what());
  }
  return 0;
}

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "skillermain");
  ros::NodeHandle n;

  skiller = new SkillerMain(n);
  return skiller->run();
}
