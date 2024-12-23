// Code generated by protoc-gen-go. DO NOT EDIT.
// versions:
// 	protoc-gen-go v1.28.1
// 	protoc        v3.20.3
// source: grSim_Commands.proto

package pb_gen

import (
	protoreflect "google.golang.org/protobuf/reflect/protoreflect"
	protoimpl "google.golang.org/protobuf/runtime/protoimpl"
	reflect "reflect"
	sync "sync"
)

const (
	// Verify that this generated code is sufficiently up-to-date.
	_ = protoimpl.EnforceVersion(20 - protoimpl.MinVersion)
	// Verify that runtime/protoimpl is sufficiently up-to-date.
	_ = protoimpl.EnforceVersion(protoimpl.MaxVersion - 20)
)

type GrSim_Robot_Command struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	Id          *uint32  `protobuf:"varint,1,req,name=id" json:"id,omitempty"`
	Kickspeedx  *float32 `protobuf:"fixed32,2,req,name=kickspeedx" json:"kickspeedx,omitempty"`
	Kickspeedz  *float32 `protobuf:"fixed32,3,req,name=kickspeedz" json:"kickspeedz,omitempty"`
	Veltangent  *float32 `protobuf:"fixed32,4,req,name=veltangent" json:"veltangent,omitempty"`
	Velnormal   *float32 `protobuf:"fixed32,5,req,name=velnormal" json:"velnormal,omitempty"`
	Velangular  *float32 `protobuf:"fixed32,6,req,name=velangular" json:"velangular,omitempty"`
	Spinner     *bool    `protobuf:"varint,7,req,name=spinner" json:"spinner,omitempty"`
	Wheelsspeed *bool    `protobuf:"varint,8,req,name=wheelsspeed" json:"wheelsspeed,omitempty"`
	Wheel1      *float32 `protobuf:"fixed32,9,opt,name=wheel1" json:"wheel1,omitempty"`
	Wheel2      *float32 `protobuf:"fixed32,10,opt,name=wheel2" json:"wheel2,omitempty"`
	Wheel3      *float32 `protobuf:"fixed32,11,opt,name=wheel3" json:"wheel3,omitempty"`
	Wheel4      *float32 `protobuf:"fixed32,12,opt,name=wheel4" json:"wheel4,omitempty"`
}

func (x *GrSim_Robot_Command) Reset() {
	*x = GrSim_Robot_Command{}
	if protoimpl.UnsafeEnabled {
		mi := &file_grSim_Commands_proto_msgTypes[0]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *GrSim_Robot_Command) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*GrSim_Robot_Command) ProtoMessage() {}

func (x *GrSim_Robot_Command) ProtoReflect() protoreflect.Message {
	mi := &file_grSim_Commands_proto_msgTypes[0]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use GrSim_Robot_Command.ProtoReflect.Descriptor instead.
func (*GrSim_Robot_Command) Descriptor() ([]byte, []int) {
	return file_grSim_Commands_proto_rawDescGZIP(), []int{0}
}

func (x *GrSim_Robot_Command) GetId() uint32 {
	if x != nil && x.Id != nil {
		return *x.Id
	}
	return 0
}

func (x *GrSim_Robot_Command) GetKickspeedx() float32 {
	if x != nil && x.Kickspeedx != nil {
		return *x.Kickspeedx
	}
	return 0
}

func (x *GrSim_Robot_Command) GetKickspeedz() float32 {
	if x != nil && x.Kickspeedz != nil {
		return *x.Kickspeedz
	}
	return 0
}

func (x *GrSim_Robot_Command) GetVeltangent() float32 {
	if x != nil && x.Veltangent != nil {
		return *x.Veltangent
	}
	return 0
}

func (x *GrSim_Robot_Command) GetVelnormal() float32 {
	if x != nil && x.Velnormal != nil {
		return *x.Velnormal
	}
	return 0
}

func (x *GrSim_Robot_Command) GetVelangular() float32 {
	if x != nil && x.Velangular != nil {
		return *x.Velangular
	}
	return 0
}

func (x *GrSim_Robot_Command) GetSpinner() bool {
	if x != nil && x.Spinner != nil {
		return *x.Spinner
	}
	return false
}

func (x *GrSim_Robot_Command) GetWheelsspeed() bool {
	if x != nil && x.Wheelsspeed != nil {
		return *x.Wheelsspeed
	}
	return false
}

func (x *GrSim_Robot_Command) GetWheel1() float32 {
	if x != nil && x.Wheel1 != nil {
		return *x.Wheel1
	}
	return 0
}

func (x *GrSim_Robot_Command) GetWheel2() float32 {
	if x != nil && x.Wheel2 != nil {
		return *x.Wheel2
	}
	return 0
}

func (x *GrSim_Robot_Command) GetWheel3() float32 {
	if x != nil && x.Wheel3 != nil {
		return *x.Wheel3
	}
	return 0
}

func (x *GrSim_Robot_Command) GetWheel4() float32 {
	if x != nil && x.Wheel4 != nil {
		return *x.Wheel4
	}
	return 0
}

type GrSim_Commands struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	Timestamp     *float64               `protobuf:"fixed64,1,req,name=timestamp" json:"timestamp,omitempty"`
	Isteamyellow  *bool                  `protobuf:"varint,2,req,name=isteamyellow" json:"isteamyellow,omitempty"`
	RobotCommands []*GrSim_Robot_Command `protobuf:"bytes,3,rep,name=robot_commands,json=robotCommands" json:"robot_commands,omitempty"`
}

func (x *GrSim_Commands) Reset() {
	*x = GrSim_Commands{}
	if protoimpl.UnsafeEnabled {
		mi := &file_grSim_Commands_proto_msgTypes[1]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *GrSim_Commands) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*GrSim_Commands) ProtoMessage() {}

func (x *GrSim_Commands) ProtoReflect() protoreflect.Message {
	mi := &file_grSim_Commands_proto_msgTypes[1]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use GrSim_Commands.ProtoReflect.Descriptor instead.
func (*GrSim_Commands) Descriptor() ([]byte, []int) {
	return file_grSim_Commands_proto_rawDescGZIP(), []int{1}
}

func (x *GrSim_Commands) GetTimestamp() float64 {
	if x != nil && x.Timestamp != nil {
		return *x.Timestamp
	}
	return 0
}

func (x *GrSim_Commands) GetIsteamyellow() bool {
	if x != nil && x.Isteamyellow != nil {
		return *x.Isteamyellow
	}
	return false
}

func (x *GrSim_Commands) GetRobotCommands() []*GrSim_Robot_Command {
	if x != nil {
		return x.RobotCommands
	}
	return nil
}

var File_grSim_Commands_proto protoreflect.FileDescriptor

var file_grSim_Commands_proto_rawDesc = []byte{
	0x0a, 0x14, 0x67, 0x72, 0x53, 0x69, 0x6d, 0x5f, 0x43, 0x6f, 0x6d, 0x6d, 0x61, 0x6e, 0x64, 0x73,
	0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x22, 0xdf, 0x02, 0x0a, 0x13, 0x67, 0x72, 0x53, 0x69, 0x6d,
	0x5f, 0x52, 0x6f, 0x62, 0x6f, 0x74, 0x5f, 0x43, 0x6f, 0x6d, 0x6d, 0x61, 0x6e, 0x64, 0x12, 0x0e,
	0x0a, 0x02, 0x69, 0x64, 0x18, 0x01, 0x20, 0x02, 0x28, 0x0d, 0x52, 0x02, 0x69, 0x64, 0x12, 0x1e,
	0x0a, 0x0a, 0x6b, 0x69, 0x63, 0x6b, 0x73, 0x70, 0x65, 0x65, 0x64, 0x78, 0x18, 0x02, 0x20, 0x02,
	0x28, 0x02, 0x52, 0x0a, 0x6b, 0x69, 0x63, 0x6b, 0x73, 0x70, 0x65, 0x65, 0x64, 0x78, 0x12, 0x1e,
	0x0a, 0x0a, 0x6b, 0x69, 0x63, 0x6b, 0x73, 0x70, 0x65, 0x65, 0x64, 0x7a, 0x18, 0x03, 0x20, 0x02,
	0x28, 0x02, 0x52, 0x0a, 0x6b, 0x69, 0x63, 0x6b, 0x73, 0x70, 0x65, 0x65, 0x64, 0x7a, 0x12, 0x1e,
	0x0a, 0x0a, 0x76, 0x65, 0x6c, 0x74, 0x61, 0x6e, 0x67, 0x65, 0x6e, 0x74, 0x18, 0x04, 0x20, 0x02,
	0x28, 0x02, 0x52, 0x0a, 0x76, 0x65, 0x6c, 0x74, 0x61, 0x6e, 0x67, 0x65, 0x6e, 0x74, 0x12, 0x1c,
	0x0a, 0x09, 0x76, 0x65, 0x6c, 0x6e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x18, 0x05, 0x20, 0x02, 0x28,
	0x02, 0x52, 0x09, 0x76, 0x65, 0x6c, 0x6e, 0x6f, 0x72, 0x6d, 0x61, 0x6c, 0x12, 0x1e, 0x0a, 0x0a,
	0x76, 0x65, 0x6c, 0x61, 0x6e, 0x67, 0x75, 0x6c, 0x61, 0x72, 0x18, 0x06, 0x20, 0x02, 0x28, 0x02,
	0x52, 0x0a, 0x76, 0x65, 0x6c, 0x61, 0x6e, 0x67, 0x75, 0x6c, 0x61, 0x72, 0x12, 0x18, 0x0a, 0x07,
	0x73, 0x70, 0x69, 0x6e, 0x6e, 0x65, 0x72, 0x18, 0x07, 0x20, 0x02, 0x28, 0x08, 0x52, 0x07, 0x73,
	0x70, 0x69, 0x6e, 0x6e, 0x65, 0x72, 0x12, 0x20, 0x0a, 0x0b, 0x77, 0x68, 0x65, 0x65, 0x6c, 0x73,
	0x73, 0x70, 0x65, 0x65, 0x64, 0x18, 0x08, 0x20, 0x02, 0x28, 0x08, 0x52, 0x0b, 0x77, 0x68, 0x65,
	0x65, 0x6c, 0x73, 0x73, 0x70, 0x65, 0x65, 0x64, 0x12, 0x16, 0x0a, 0x06, 0x77, 0x68, 0x65, 0x65,
	0x6c, 0x31, 0x18, 0x09, 0x20, 0x01, 0x28, 0x02, 0x52, 0x06, 0x77, 0x68, 0x65, 0x65, 0x6c, 0x31,
	0x12, 0x16, 0x0a, 0x06, 0x77, 0x68, 0x65, 0x65, 0x6c, 0x32, 0x18, 0x0a, 0x20, 0x01, 0x28, 0x02,
	0x52, 0x06, 0x77, 0x68, 0x65, 0x65, 0x6c, 0x32, 0x12, 0x16, 0x0a, 0x06, 0x77, 0x68, 0x65, 0x65,
	0x6c, 0x33, 0x18, 0x0b, 0x20, 0x01, 0x28, 0x02, 0x52, 0x06, 0x77, 0x68, 0x65, 0x65, 0x6c, 0x33,
	0x12, 0x16, 0x0a, 0x06, 0x77, 0x68, 0x65, 0x65, 0x6c, 0x34, 0x18, 0x0c, 0x20, 0x01, 0x28, 0x02,
	0x52, 0x06, 0x77, 0x68, 0x65, 0x65, 0x6c, 0x34, 0x22, 0x8f, 0x01, 0x0a, 0x0e, 0x67, 0x72, 0x53,
	0x69, 0x6d, 0x5f, 0x43, 0x6f, 0x6d, 0x6d, 0x61, 0x6e, 0x64, 0x73, 0x12, 0x1c, 0x0a, 0x09, 0x74,
	0x69, 0x6d, 0x65, 0x73, 0x74, 0x61, 0x6d, 0x70, 0x18, 0x01, 0x20, 0x02, 0x28, 0x01, 0x52, 0x09,
	0x74, 0x69, 0x6d, 0x65, 0x73, 0x74, 0x61, 0x6d, 0x70, 0x12, 0x22, 0x0a, 0x0c, 0x69, 0x73, 0x74,
	0x65, 0x61, 0x6d, 0x79, 0x65, 0x6c, 0x6c, 0x6f, 0x77, 0x18, 0x02, 0x20, 0x02, 0x28, 0x08, 0x52,
	0x0c, 0x69, 0x73, 0x74, 0x65, 0x61, 0x6d, 0x79, 0x65, 0x6c, 0x6c, 0x6f, 0x77, 0x12, 0x3b, 0x0a,
	0x0e, 0x72, 0x6f, 0x62, 0x6f, 0x74, 0x5f, 0x63, 0x6f, 0x6d, 0x6d, 0x61, 0x6e, 0x64, 0x73, 0x18,
	0x03, 0x20, 0x03, 0x28, 0x0b, 0x32, 0x14, 0x2e, 0x67, 0x72, 0x53, 0x69, 0x6d, 0x5f, 0x52, 0x6f,
	0x62, 0x6f, 0x74, 0x5f, 0x43, 0x6f, 0x6d, 0x6d, 0x61, 0x6e, 0x64, 0x52, 0x0d, 0x72, 0x6f, 0x62,
	0x6f, 0x74, 0x43, 0x6f, 0x6d, 0x6d, 0x61, 0x6e, 0x64, 0x73, 0x42, 0x2d, 0x5a, 0x2b, 0x67, 0x69,
	0x74, 0x68, 0x75, 0x62, 0x2e, 0x63, 0x6f, 0x6d, 0x2f, 0x52, 0x69, 0x6f, 0x6e, 0x65, 0x2d, 0x53,
	0x53, 0x4c, 0x2f, 0x52, 0x41, 0x43, 0x4f, 0x4f, 0x4e, 0x2d, 0x50, 0x69, 0x2f, 0x70, 0x72, 0x6f,
	0x74, 0x6f, 0x2f, 0x70, 0x62, 0x5f, 0x67, 0x65, 0x6e,
}

var (
	file_grSim_Commands_proto_rawDescOnce sync.Once
	file_grSim_Commands_proto_rawDescData = file_grSim_Commands_proto_rawDesc
)

func file_grSim_Commands_proto_rawDescGZIP() []byte {
	file_grSim_Commands_proto_rawDescOnce.Do(func() {
		file_grSim_Commands_proto_rawDescData = protoimpl.X.CompressGZIP(file_grSim_Commands_proto_rawDescData)
	})
	return file_grSim_Commands_proto_rawDescData
}

var file_grSim_Commands_proto_msgTypes = make([]protoimpl.MessageInfo, 2)
var file_grSim_Commands_proto_goTypes = []interface{}{
	(*GrSim_Robot_Command)(nil), // 0: grSim_Robot_Command
	(*GrSim_Commands)(nil),      // 1: grSim_Commands
}
var file_grSim_Commands_proto_depIdxs = []int32{
	0, // 0: grSim_Commands.robot_commands:type_name -> grSim_Robot_Command
	1, // [1:1] is the sub-list for method output_type
	1, // [1:1] is the sub-list for method input_type
	1, // [1:1] is the sub-list for extension type_name
	1, // [1:1] is the sub-list for extension extendee
	0, // [0:1] is the sub-list for field type_name
}

func init() { file_grSim_Commands_proto_init() }
func file_grSim_Commands_proto_init() {
	if File_grSim_Commands_proto != nil {
		return
	}
	if !protoimpl.UnsafeEnabled {
		file_grSim_Commands_proto_msgTypes[0].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*GrSim_Robot_Command); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_grSim_Commands_proto_msgTypes[1].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*GrSim_Commands); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
	}
	type x struct{}
	out := protoimpl.TypeBuilder{
		File: protoimpl.DescBuilder{
			GoPackagePath: reflect.TypeOf(x{}).PkgPath(),
			RawDescriptor: file_grSim_Commands_proto_rawDesc,
			NumEnums:      0,
			NumMessages:   2,
			NumExtensions: 0,
			NumServices:   0,
		},
		GoTypes:           file_grSim_Commands_proto_goTypes,
		DependencyIndexes: file_grSim_Commands_proto_depIdxs,
		MessageInfos:      file_grSim_Commands_proto_msgTypes,
	}.Build()
	File_grSim_Commands_proto = out.File
	file_grSim_Commands_proto_rawDesc = nil
	file_grSim_Commands_proto_goTypes = nil
	file_grSim_Commands_proto_depIdxs = nil
}
