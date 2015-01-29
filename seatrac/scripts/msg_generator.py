#!/usr/bin/env python
from snippets.generators.cppgen.structure import Structure
from snippets.generators.cppgen.function import Function
from snippets.generators.cppgen.variable import Variable
from snippets.generators.cppgen import struct_serializer
import xml.etree.ElementTree as ET

def gen_factory_initializer(structs, bindent = '', indent = '  '):
    """
    Generates the factory map initializer.
        
    bindent -- the current indent in the document
    indent -- the usual minimum indentation
    """
    code = ''
    for struct in structs:
        code = code + indent + '{' + struct.sname + '::CID, createInstance<' + struct.sname + '>},\n'   
    
    #Remove the last comma and newline
    return code[:-2]

def create_datatypes(xmlroot, folder = ''):
    structs = []
    for node in xmlroot.findall('struct'):
        structs.append(Structure(node))
        
    fdt = open(folder + '/' + 'datatype_defs.h','w')
    for struct in structs:
        fdt.write(struct.gen_code(indent=''))
        fdt.write('\n')
        
    fdt.close()
    
    return structs
          
def create_messages(xmlroot, prefix, folder = ''):
    structs = []
    for node in xmlroot.findall('struct'):
        s = Structure(node)
        #'''Add automatic CID field'''
        #v = Variable()
        #v.vname = 'cid'
        #v.vtype = 'uint8_t' 
        #s.svariables.insert(0,v)

        '''Add automatic inheritance and virtual method implementations'''
        s.sinherit = 'SeatracMessage'
        #Ptr typedef
        t = ('boost::shared_ptr< ' + s.sname +' > ', 'Ptr')     
        s.stypedefs.append(t)  
        t = ('boost::shared_ptr< ' + s.sname +' const > ', 'ConstPtr') 
        s.stypedefs.append(t)  
        #get_cid function
        f = Function()
        f.fname = 'getCid'
        f.fret = 'int'
        f.fbody = 'return ' + s.sname + '::CID;'
        f.finline = True
        f.fqual = 'const'
        s.smethods.append(f)
                
        #serialize function
        f = Function()
        f.fname = 'pack'
        f.fret = 'bool'
        f.fbody = 'return seatrac_serialize(this, out);'
        f.fqual = 'const'
        a1 = Variable()
        a1.vname = 'out'
        a1.vtype = 'SeatracMessage::DataBuffer&'
        f.fargs.append(a1)
        s.smethods.append(f)
        
        #serialize function
        f = Function()
        f.fname = 'unpack'
        f.fret = 'bool'
        f.fbody = 'return seatrac_deserialize(this, in);'
        a1 = Variable()
        a1.vname = 'in'
        a1.vtype = 'const SeatracMessage::DataBuffer&'
        f.fargs.append(a1)
        s.smethods.append(f)          
        
        structs.append(s)
    
    filebase = folder + '/' + prefix
    
    cmi = open(filebase + '_factory_initializer.h','w')
    cmi.write(gen_factory_initializer(structs))
    cmi.close()
    
    smsg = open(filebase + '_defs.h','w')
    for struct in structs:
        smsg.write(struct.gen_code(indent=''))
        smsg.write('\n')
    smsg.close()
    
    smsgimpl = open(filebase + '_impl.h','w')
    for struct in structs:
        smsgimpl.write(struct.gen_impl(indent=''))
        smsgimpl.write('\n')
    smsgimpl.close()
    
    return structs
               

if __name__ == '__main__':
    structs = [];
    '''Parse the Seatrac data types and create the definitions'''
    dt = create_datatypes(ET.parse('definitions/SeatracDataTypes.xml').getroot(),
                          'include/labust/seatrac/detail')
    '''Parse the Seatrac commands and create the initializer'''
    cmd = create_messages(ET.parse('definitions/SeatracCommands.xml').getroot(), 
                          'command',
                          'include/labust/seatrac/detail')
    '''Parse the Seatrac responses and create the initializer'''
    resp = create_messages(ET.parse('definitions/SeatracResponses.xml').getroot(), 
                           'response',
                           'include/labust/seatrac/detail')  
    
    structs.extend(dt);
    structs.extend(cmd);
    structs.extend(resp);
  
    '''Create boost serializator for all defined structures'''  
    serdef = open('include/labust/seatrac/detail/serialization_defs.h','w')
    for struct in structs:
        serdef.write(struct_serializer.gen_serializer(struct, ns='labust::seatrac'))
        serdef.write('\n')
        
