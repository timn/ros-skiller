
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

#include <lua_utils/fam.h>
#include <lua_utils/context.h>


class SkillerMain
{
 public:
  SkillerMain(ros::NodeHandle &n)
    : __lua(/* watch files */ true), __n(n)
  {
    std::string skill_space = "test";
    if (n.hasParam("/skiller/skillspace")) {
      n.getParam("/skiller/skillspace", skill_space);
    }

    __lua.add_package("roslua");
    __lua.set_string("SKILLSPACE", skill_space.c_str());

    // init Lua node
    __lua.do_string("roslua.init_node{master_uri=\"%s\", node_name=\"/skiller\"}",
		    ros::master::getURI().c_str());

    __lua.set_start_script(LUADIR"/skiller/start.lua");
  }

  ~SkillerMain()
  {
  }

  int run()
  {
    bool quit = false;
    // run until skiller stopped
    __lua.get_global("roslua");		// roslua
    while (! quit && __n.ok() ) {
      try {
	// Spin!
	__lua.get_field(-1, "spin");	// roslua roslua.spin
	try {
	  __lua.pcall();	// roslua
	} catch (Exception &e) {
	  printf("%s", e.what());
	}

	// get quite flag
	__lua.get_field(-1, "quit");	// roslua roslua.quit
	quit = __lua.to_boolean(-1);
	__lua.pop(1);			// roslua
      } catch (Exception &e) {
	printf("%s\n", e.what());
      }
    }
    __lua.get_field(-1, "finalize");	// roslua roslua.finalize
    __lua.pcall();			// roslua
    __lua.pop(1);			// ---
    return 0;
  }

 private:
  fawkes::LuaContext __lua;
  ros::NodeHandle &__n;
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "skillermain");
  ros::NodeHandle n;

  SkillerMain skiller(n);
  return skiller.run();
}
