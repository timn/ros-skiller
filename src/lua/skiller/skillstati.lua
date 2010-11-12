
----------------------------------------------------------------------------
--  skillstati.lua - Possible skill return values
--
--  Created: Fri Dec 12 18:55:14 2008
--  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
--             2010       Carnegie Mellon University
--             2010       Intel Labs Pittsburgh
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

module(...)

S_INACTIVE = 0
S_FINAL    = 1
S_RUNNING  = 2
S_FAILED   = 3

status_tostring = {}
status_tostring[0] = "S_INACTIVE"
status_tostring[1] = "S_FINAL"
status_tostring[2] = "S_RUNNING"
status_tostring[3] = "S_FAILED"
