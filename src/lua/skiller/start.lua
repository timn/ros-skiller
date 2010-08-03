
----------------------------------------------------------------------------
--  start.lua - Skiller startup script -- ROS version
--
--  Created: Tue Aug  3 15:29:28 2010 (at Intel Research, Pittsburgh)
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
--
----------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.

local topic = "/chatter"
local msgtype = "std_msgs/String"

local s = roslua.subscriber(topic, msgtype)
s:add_listener(function (message)
		  message:print()
	       end)
