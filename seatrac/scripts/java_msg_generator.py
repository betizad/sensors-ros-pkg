#!/usr/bin/env python
from snippets.generators.javagen.javagen import JavaGen
from snippets.generators.elements.structure import Structure
from snippets.generators.elements.function import Function
from snippets.generators.elements.variable import Variable
import xml.etree.ElementTree as ET

def gen_factory_initializer(cmds, resp, folder, bindent = '', indent = '  '):
    """
    Generates the factory initializer.
        
    bindent -- the current indent in the document
    indent -- the usual minimum indentation
    """
    s = Structure()
    s.name = 'SeatracFactory'
    
    f = Function()
    f.ret = 'SeatracMessage'
    f.name = 'createCommand'
    f.qual = 'throws RuntimeException'
    f.static = True
    v = Variable()
    v.type = 'int'
    v.name = 'cid'
    f.args.append(v)
    
    code = '  switch (cid){\n'
    for struct in cmds:
        code = code + '    case ' + struct.name + '.CID: return new ' + struct.name + '();\n'
    code = code + '    default: throw new RuntimeException("No command with CID=" + cid);\n }\n'
    f.body = code;
    s.methods.append(f)
    
    f = Function()
    f.ret = 'SeatracMessage'
    f.name = 'createResponse'
    f.qual = 'throws RuntimeException'
    f.static = True
    v = Variable()
    v.type = 'int'
    v.name = 'cid'
    f.args.append(v)
    
    code = '  switch (cid){\n'
    for struct in resp:
        code = code + '    case ' + struct.name + '.CID: return new ' + struct.name + '();\n'
    code = code + '    default: throw new RuntimeException("No responseSeatracFactory.java with CID=" + cid);\n }\n'
    f.body = code;
    s.methods.append(f)
    
    f = Function()
    f.ret = 'String'
    f.name = 'getCommandName'
    f.qual = 'throws RuntimeException'
    f.static = True
    v = Variable()
    v.type = 'int'
    v.name = 'cid'
    f.args.append(v)
    
    code = '  switch (cid){\n'
    for struct in cmds:
        code = code + '    case ' + struct.name + '.CID: return "' + struct.name + '";\n'
    code = code + '    default: throw new RuntimeException("No command with CID=" + cid);\n }\n'
    f.body = code;
    s.methods.append(f)
    
    f = Function()
    f.ret = 'String'
    f.name = 'getResponseName'
    f.qual = 'throws RuntimeException'
    f.static = True
    v = Variable()
    v.type = 'int'
    v.name = 'cid'
    f.args.append(v)
    
    code = '  switch (cid){\n'
    for struct in cmds:
        code = code + '    case ' + struct.name + '.CID: return "' + struct.name + '";\n'
    code = code + '    default: throw new RuntimeException("No response with CID=" + cid);\n }\n'
    f.body = code;
    s.methods.append(f)   
    
    fdt = open(folder + '/' + s.name + '.java','w')
    imports = [
     'hr.fer.labust.seatrac.messages.commands.*',
     'hr.fer.labust.seatrac.messages.responses.*'
    ]
    fdt.write(gen.gen_class(s,'hr.fer.labust.seatrac', imports, False))
    fdt.write('\n')
    fdt.close() 
    
    

def gen_names(structs, bindent = '', indent = '  '):
    """
    Generates the human readable names of CIDs.
        
    bindent -- the current indent in the document
    indent -- the usual minimum indentation
    """
    code = ''
    for struct in structs:
        code = code + indent + '{' + struct.sname + '::CID, \"' + struct.sname + '\"},\n'   
    
    #Remove the last comma and newline
    return code[:-2]

def create_datatypes(xmlroot, gen, folder = ''):
    structs = []
        
    for node in xmlroot.findall('struct'):
        structs.append(Structure(node))
       
    for struct in structs:
        gen.packable_types.append(struct.name)
    
    for struct in structs:
        fdt = open(folder + '/' + struct.name + '.java','w')
        fdt.write(gen.gen_class(struct,'hr.fer.labust.seatrac.datatypes'))
        fdt.write('\n')
        fdt.close()
    
    return structs
          
def create_messages(xmlroot, prefix, gen, folder = ''):
    structs = []
    for node in xmlroot.findall('struct'):
        structs.append(Structure(node))
    
    for struct in structs:
        gen.packable_types.append(struct.name)
        
    for s in structs:
        '''Add automatic inheritance and virtual method implementations'''
        s.inherit = 'SeatracMessage'

        #getCid function
        f = Function()
        f.name = 'getCid'
        f.ret = 'int'
        f.body = 'return ' + s.name + '.CID;'
        s.methods.append(f)
        
        #Test message type
        f = Function()
        f.name = 'isCommand'
        f.ret = 'bool'
        if prefix == "command":
            f.body = 'return 1;'
        else:
            f.body = 'return 0;'
        s.methods.append(f)

        fdt = open(folder + '/' + s.name + '.java','w')
        imports = [
         'hr.fer.labust.seatrac.datatypes.*',
         'hr.fer.labust.seatrac.SeatracMessage'
        ]
        fdt.write(gen.gen_class(s,'hr.fer.labust.seatrac.messages.' + prefix + 's', imports))
        fdt.write('\n')
        fdt.close()      
                   
    return structs
               

if __name__ == '__main__':
    structs = [];
    
    gen = JavaGen()
    gen.array_len_type = 'byte'
    gen.type_equiv['vec3si'] = 'short[3]'
    gen.type_equiv['USBLRSSIVec'] = 'short[]'
    gen.type_equiv['PayloadType'] = 'byte[]'
    
    '''Parse the Seatrac data types and create the definitions'''
    dt = create_datatypes(ET.parse('definitions/SeatracDataTypes.xml').getroot(), gen,
                          'java/hr/fer/labust/seatrac/datatypes')
    '''Parse the Seatrac commands and create the initializer'''
    cmd = create_messages(ET.parse('definitions/SeatracCommands.xml').getroot(), 
                          'command', gen,
                          'java/hr/fer/labust/seatrac/messages/commands')
    '''Parse the Seatrac responses and create the initializer'''
    resp = create_messages(ET.parse('definitions/SeatracResponses.xml').getroot(), 
                           'response', gen,
                           'java/hr/fer/labust/seatrac/messages/responses')  
    
    factstructs = []
    factstructs.extend(cmd)
    factstructs.extend(resp)
    
    gen_factory_initializer(cmd, resp, 'java/hr/fer/labust/seatrac/')
    
    exit()
    
    structs.extend(dt);
    structs.extend(cmd);
    structs.extend(resp);
  
    '''Create boost serializator for all defined structures'''  
    serdef = open('include/labust/seatrac/detail/serialization_defs.h','w')
    for struct in structs:
        serdef.write(struct_serializer.gen_serializer(struct, ns='labust::seatrac'))
        serdef.write('\n')
        
