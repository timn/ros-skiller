
----------------------------------------------------------------------------
--  log_mongo.lua - Log skill execution to MongoDB
--
--  Created: Mon Nov 01 18:19:58 2010
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

require("fawkes.modinit")
module("skiller.log_mongo", fawkes.modinit.register_all)

local mongo = require("mongo")

local db
local default_host = "localhost"

function init(host)
   local host = host or default_host
   db = assert(mongo.Connection.New{auto_reconnect=true})
   assert(db:connect(host))
end


function insert(collection, document)
   local ok, err = pcall(db.insert, db, collection, document)
   if not ok then
      print_warn("Failed to insert document to %s: %s", collection, err)
   end
end

function update(collection, query, document, upsert, multi)
   local ok, err = pcall(db.update, db, collection, query, document, upsert, multi)
   if not ok then
      print_warn("Failed to update document in %s: %s", collection, err)
   end
end

function now()
   return mongo.Date(os.time() * 1000)
end
