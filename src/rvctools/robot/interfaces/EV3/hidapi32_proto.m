function [methodinfo,structs,enuminfo,ThunkLibName]=hidapi32_proto
%HIDAPI32_PROTO Create structures to define interfaces found in 'hidapi'.

%This function was generated by loadlibrary.m parser version 1.1.6.34 on Thu Jan 23 15:02:05 2014
%perl options:'hidapi.i -outfile=hidapi32_proto.m'
ival={cell(1,0)}; % change 0 to the actual number of functions to preallocate the data.
structs=[];enuminfo=[];fcnNum=1;
fcns=struct('name',ival,'calltype',ival,'LHS',ival,'RHS',ival,'alias',ival);
ThunkLibName=[];
% int  hid_init ( void ); 
fcns.name{fcnNum}='hid_init'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}=[];fcnNum=fcnNum+1;
% int  hid_exit ( void ); 
fcns.name{fcnNum}='hid_exit'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}=[];fcnNum=fcnNum+1;
% struct hid_device_info  * hid_enumerate ( unsigned short vendor_id , unsigned short product_id ); 
fcns.name{fcnNum}='hid_enumerate'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='hid_device_infoPtr'; fcns.RHS{fcnNum}={'uint16', 'uint16'};fcnNum=fcnNum+1;
% void  hid_free_enumeration ( struct hid_device_info * devs ); 
fcns.name{fcnNum}='hid_free_enumeration'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}=[]; fcns.RHS{fcnNum}={'hid_device_infoPtr'};fcnNum=fcnNum+1;
%  hid_device * hid_open ( unsigned short vendor_id , unsigned short product_id , wchar_t * serial_number ); 
fcns.name{fcnNum}='hid_open'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='hid_device_Ptr'; fcns.RHS{fcnNum}={'uint16', 'uint16', 'uint16Ptr'};fcnNum=fcnNum+1;
%  hid_device * hid_open_path ( const char * path ); 
fcns.name{fcnNum}='hid_open_path'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='hid_device_Ptr'; fcns.RHS{fcnNum}={'cstring'};fcnNum=fcnNum+1;
% int  hid_write ( hid_device * device , const unsigned char * data , size_t length ); 
fcns.name{fcnNum}='hid_write'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}={'hid_device_Ptr', 'uint8Ptr', 'uint32'};fcnNum=fcnNum+1;
% int  hid_read_timeout ( hid_device * dev , unsigned char * data , size_t length , int milliseconds ); 
fcns.name{fcnNum}='hid_read_timeout'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}={'hid_device_Ptr', 'uint8Ptr', 'uint32', 'int32'};fcnNum=fcnNum+1;
% int  hid_read ( hid_device * device , unsigned char * data , size_t length ); 
fcns.name{fcnNum}='hid_read'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}={'hid_device_Ptr', 'uint8Ptr', 'uint32'};fcnNum=fcnNum+1;
% int  hid_set_nonblocking ( hid_device * device , int nonblock ); 
fcns.name{fcnNum}='hid_set_nonblocking'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}={'hid_device_Ptr', 'int32'};fcnNum=fcnNum+1;
% int  hid_send_feature_report ( hid_device * device , const unsigned char * data , size_t length ); 
fcns.name{fcnNum}='hid_send_feature_report'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}={'hid_device_Ptr', 'uint8Ptr', 'uint32'};fcnNum=fcnNum+1;
% int  hid_get_feature_report ( hid_device * device , unsigned char * data , size_t length ); 
fcns.name{fcnNum}='hid_get_feature_report'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}={'hid_device_Ptr', 'uint8Ptr', 'uint32'};fcnNum=fcnNum+1;
% void  hid_close ( hid_device * device ); 
fcns.name{fcnNum}='hid_close'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}=[]; fcns.RHS{fcnNum}={'hid_device_Ptr'};fcnNum=fcnNum+1;
% int  hid_get_manufacturer_string ( hid_device * device , wchar_t * string , size_t maxlen ); 
fcns.name{fcnNum}='hid_get_manufacturer_string'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}={'hid_device_Ptr', 'uint16Ptr', 'uint32'};fcnNum=fcnNum+1;
% int  hid_get_product_string ( hid_device * device , wchar_t * string , size_t maxlen ); 
fcns.name{fcnNum}='hid_get_product_string'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}={'hid_device_Ptr', 'uint16Ptr', 'uint32'};fcnNum=fcnNum+1;
% int  hid_get_serial_number_string ( hid_device * device , wchar_t * string , size_t maxlen ); 
fcns.name{fcnNum}='hid_get_serial_number_string'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}={'hid_device_Ptr', 'uint16Ptr', 'uint32'};fcnNum=fcnNum+1;
% int  hid_get_indexed_string ( hid_device * device , int string_index , wchar_t * string , size_t maxlen ); 
fcns.name{fcnNum}='hid_get_indexed_string'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='int32'; fcns.RHS{fcnNum}={'hid_device_Ptr', 'int32', 'uint16Ptr', 'uint32'};fcnNum=fcnNum+1;
%  const wchar_t * hid_error ( hid_device * device ); 
fcns.name{fcnNum}='hid_error'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint16Ptr'; fcns.RHS{fcnNum}={'hid_device_Ptr'};fcnNum=fcnNum+1;
structs.hid_device_.members=struct('');
structs.hid_device_info.members=struct('path', 'cstring', 'vendor_id', 'uint16', 'product_id', 'uint16', 'serial_number', 'uint16Ptr', 'release_number', 'uint16', 'manufacturer_string', 'uint16Ptr', 'product_string', 'uint16Ptr', 'usage_page', 'uint16', 'usage', 'uint16', 'interface_number', 'int32', 'next', 'hid_device_infoPtr');
methodinfo=fcns;