import json
import dataclasses
from cxxheaderparser.types import Array, Pointer
from cxxheaderparser.simple import parse_string

prefix  = """// THIS FILE IS AUTO GENERATED - SEE parse_headers.py FOR HOW IT GETS GENERATED!
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "mujoco/mujoco.h"

#include <emscripten/fetch.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <vector>

using namespace emscripten;

"""

class_definitions      = ""
emscripten_bindings    = "\n\nEMSCRIPTEN_BINDINGS(mujoco_wasm) {\n\n"
typescript_definitions = ""

with open("include/mujoco/mjmodel.h") as f:
    content = f.read()
    parsed_data = parse_string(content)

    with open("parsed_json.json", mode='w') as j:
        json.dump(dataclasses.asdict(parsed_data), j, indent=2)

for data in parsed_data.namespace.classes:
    if data.class_decl.classkey == "struct":
        struct_name = data.class_decl.typename.segments[0].name
        if struct_name.endswith("_"):
            struct_name = struct_name[:-1]
        print("Struct Name:", struct_name)
        class_definitions += "class "+struct_name[2:]+" {\npublic:\n  "+struct_name[2:]+'() { m = NULL; }\n\n  '+struct_name+' *ptr  () { return m; }\n  '+struct_name+' getVal() { return *m; }\n\n'
        emscripten_bindings += '  class_<'+struct_name[2:]+'>("'+struct_name[2:].ljust(22)+'")\n      .constructor<>()\n'
        for field in data.fields:
            field_name = field.name
            if field.access == "public" and not "Visual" in struct_name:
                emscripten_bindings += '      .function('+('"'+field_name+'"').ljust(22)+', &'+struct_name[2:]+'::'+field_name.ljust(22)+')\n'
                #print(field.type)
                field_type = "void"
                if isinstance(field.type, Array):
                    if isinstance(field.type.array_of, Array):
                        field_type = "Array of Array of " + field.type.array_of.array_of.typename.segments[0].name
                    elif isinstance(field.type.array_of, Pointer):
                        field_type = "Pointer of " + field.type.array_of.ptr_to.typename.segments[0].name
                    else:
                        field_type = "Array of " + field.type.array_of.typename.segments[0].name
                else:
                    if isinstance(field.type, Pointer):
                        field_type = "Pointer of " + field.type.ptr_to.typename.segments[0].name
                        class_definitions += '  val '+field_name.ljust(22)+'() { return val(typed_memory_view(m->nmesh, m->'+field_name.ljust(22)+' )); } ' + field.doxygen +"\n"
                    elif field.type.typename:
                        field_type = field.type.typename.segments[0].name
                        field_type = field_type.replace("mjtNum", "double")
                        class_definitions += '  '+field_type.ljust(6)+' '+field_name.ljust(22)+'() { return m->'+field_name.ljust(22)+'; } ' + field.doxygen +"\n"
                print("  Field Name:", field_name, ", Field Type:", field_type, field.doxygen)
        class_definitions += 'private:\n  '+struct_name+' *m;\n};\n'
        emscripten_bindings += ";\n\n"
emscripten_bindings += "}\n"

full_cc = prefix + class_definitions + emscripten_bindings
with open("main-genned.cc", mode="w") as f:
    f.write(full_cc)
