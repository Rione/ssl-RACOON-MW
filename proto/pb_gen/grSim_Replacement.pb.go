// Code generated by protoc-gen-go. DO NOT EDIT.
// versions:
// 	protoc-gen-go v1.28.1
// 	protoc        v3.20.3
// source: grSim_Replacement.proto

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

type GrSim_RobotReplacement struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	X          *float64 `protobuf:"fixed64,1,req,name=x" json:"x,omitempty"`
	Y          *float64 `protobuf:"fixed64,2,req,name=y" json:"y,omitempty"`
	Dir        *float64 `protobuf:"fixed64,3,req,name=dir" json:"dir,omitempty"`
	Id         *uint32  `protobuf:"varint,4,req,name=id" json:"id,omitempty"`
	Yellowteam *bool    `protobuf:"varint,5,req,name=yellowteam" json:"yellowteam,omitempty"`
	Turnon     *bool    `protobuf:"varint,6,opt,name=turnon" json:"turnon,omitempty"`
}

func (x *GrSim_RobotReplacement) Reset() {
	*x = GrSim_RobotReplacement{}
	if protoimpl.UnsafeEnabled {
		mi := &file_grSim_Replacement_proto_msgTypes[0]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *GrSim_RobotReplacement) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*GrSim_RobotReplacement) ProtoMessage() {}

func (x *GrSim_RobotReplacement) ProtoReflect() protoreflect.Message {
	mi := &file_grSim_Replacement_proto_msgTypes[0]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use GrSim_RobotReplacement.ProtoReflect.Descriptor instead.
func (*GrSim_RobotReplacement) Descriptor() ([]byte, []int) {
	return file_grSim_Replacement_proto_rawDescGZIP(), []int{0}
}

func (x *GrSim_RobotReplacement) GetX() float64 {
	if x != nil && x.X != nil {
		return *x.X
	}
	return 0
}

func (x *GrSim_RobotReplacement) GetY() float64 {
	if x != nil && x.Y != nil {
		return *x.Y
	}
	return 0
}

func (x *GrSim_RobotReplacement) GetDir() float64 {
	if x != nil && x.Dir != nil {
		return *x.Dir
	}
	return 0
}

func (x *GrSim_RobotReplacement) GetId() uint32 {
	if x != nil && x.Id != nil {
		return *x.Id
	}
	return 0
}

func (x *GrSim_RobotReplacement) GetYellowteam() bool {
	if x != nil && x.Yellowteam != nil {
		return *x.Yellowteam
	}
	return false
}

func (x *GrSim_RobotReplacement) GetTurnon() bool {
	if x != nil && x.Turnon != nil {
		return *x.Turnon
	}
	return false
}

type GrSim_BallReplacement struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	X  *float64 `protobuf:"fixed64,1,opt,name=x" json:"x,omitempty"`
	Y  *float64 `protobuf:"fixed64,2,opt,name=y" json:"y,omitempty"`
	Vx *float64 `protobuf:"fixed64,3,opt,name=vx" json:"vx,omitempty"`
	Vy *float64 `protobuf:"fixed64,4,opt,name=vy" json:"vy,omitempty"`
}

func (x *GrSim_BallReplacement) Reset() {
	*x = GrSim_BallReplacement{}
	if protoimpl.UnsafeEnabled {
		mi := &file_grSim_Replacement_proto_msgTypes[1]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *GrSim_BallReplacement) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*GrSim_BallReplacement) ProtoMessage() {}

func (x *GrSim_BallReplacement) ProtoReflect() protoreflect.Message {
	mi := &file_grSim_Replacement_proto_msgTypes[1]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use GrSim_BallReplacement.ProtoReflect.Descriptor instead.
func (*GrSim_BallReplacement) Descriptor() ([]byte, []int) {
	return file_grSim_Replacement_proto_rawDescGZIP(), []int{1}
}

func (x *GrSim_BallReplacement) GetX() float64 {
	if x != nil && x.X != nil {
		return *x.X
	}
	return 0
}

func (x *GrSim_BallReplacement) GetY() float64 {
	if x != nil && x.Y != nil {
		return *x.Y
	}
	return 0
}

func (x *GrSim_BallReplacement) GetVx() float64 {
	if x != nil && x.Vx != nil {
		return *x.Vx
	}
	return 0
}

func (x *GrSim_BallReplacement) GetVy() float64 {
	if x != nil && x.Vy != nil {
		return *x.Vy
	}
	return 0
}

type GrSim_Replacement struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	Ball   *GrSim_BallReplacement    `protobuf:"bytes,1,opt,name=ball" json:"ball,omitempty"`
	Robots []*GrSim_RobotReplacement `protobuf:"bytes,2,rep,name=robots" json:"robots,omitempty"`
}

func (x *GrSim_Replacement) Reset() {
	*x = GrSim_Replacement{}
	if protoimpl.UnsafeEnabled {
		mi := &file_grSim_Replacement_proto_msgTypes[2]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *GrSim_Replacement) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*GrSim_Replacement) ProtoMessage() {}

func (x *GrSim_Replacement) ProtoReflect() protoreflect.Message {
	mi := &file_grSim_Replacement_proto_msgTypes[2]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use GrSim_Replacement.ProtoReflect.Descriptor instead.
func (*GrSim_Replacement) Descriptor() ([]byte, []int) {
	return file_grSim_Replacement_proto_rawDescGZIP(), []int{2}
}

func (x *GrSim_Replacement) GetBall() *GrSim_BallReplacement {
	if x != nil {
		return x.Ball
	}
	return nil
}

func (x *GrSim_Replacement) GetRobots() []*GrSim_RobotReplacement {
	if x != nil {
		return x.Robots
	}
	return nil
}

var File_grSim_Replacement_proto protoreflect.FileDescriptor

var file_grSim_Replacement_proto_rawDesc = []byte{
	0x0a, 0x17, 0x67, 0x72, 0x53, 0x69, 0x6d, 0x5f, 0x52, 0x65, 0x70, 0x6c, 0x61, 0x63, 0x65, 0x6d,
	0x65, 0x6e, 0x74, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x22, 0x8e, 0x01, 0x0a, 0x16, 0x67, 0x72,
	0x53, 0x69, 0x6d, 0x5f, 0x52, 0x6f, 0x62, 0x6f, 0x74, 0x52, 0x65, 0x70, 0x6c, 0x61, 0x63, 0x65,
	0x6d, 0x65, 0x6e, 0x74, 0x12, 0x0c, 0x0a, 0x01, 0x78, 0x18, 0x01, 0x20, 0x02, 0x28, 0x01, 0x52,
	0x01, 0x78, 0x12, 0x0c, 0x0a, 0x01, 0x79, 0x18, 0x02, 0x20, 0x02, 0x28, 0x01, 0x52, 0x01, 0x79,
	0x12, 0x10, 0x0a, 0x03, 0x64, 0x69, 0x72, 0x18, 0x03, 0x20, 0x02, 0x28, 0x01, 0x52, 0x03, 0x64,
	0x69, 0x72, 0x12, 0x0e, 0x0a, 0x02, 0x69, 0x64, 0x18, 0x04, 0x20, 0x02, 0x28, 0x0d, 0x52, 0x02,
	0x69, 0x64, 0x12, 0x1e, 0x0a, 0x0a, 0x79, 0x65, 0x6c, 0x6c, 0x6f, 0x77, 0x74, 0x65, 0x61, 0x6d,
	0x18, 0x05, 0x20, 0x02, 0x28, 0x08, 0x52, 0x0a, 0x79, 0x65, 0x6c, 0x6c, 0x6f, 0x77, 0x74, 0x65,
	0x61, 0x6d, 0x12, 0x16, 0x0a, 0x06, 0x74, 0x75, 0x72, 0x6e, 0x6f, 0x6e, 0x18, 0x06, 0x20, 0x01,
	0x28, 0x08, 0x52, 0x06, 0x74, 0x75, 0x72, 0x6e, 0x6f, 0x6e, 0x22, 0x53, 0x0a, 0x15, 0x67, 0x72,
	0x53, 0x69, 0x6d, 0x5f, 0x42, 0x61, 0x6c, 0x6c, 0x52, 0x65, 0x70, 0x6c, 0x61, 0x63, 0x65, 0x6d,
	0x65, 0x6e, 0x74, 0x12, 0x0c, 0x0a, 0x01, 0x78, 0x18, 0x01, 0x20, 0x01, 0x28, 0x01, 0x52, 0x01,
	0x78, 0x12, 0x0c, 0x0a, 0x01, 0x79, 0x18, 0x02, 0x20, 0x01, 0x28, 0x01, 0x52, 0x01, 0x79, 0x12,
	0x0e, 0x0a, 0x02, 0x76, 0x78, 0x18, 0x03, 0x20, 0x01, 0x28, 0x01, 0x52, 0x02, 0x76, 0x78, 0x12,
	0x0e, 0x0a, 0x02, 0x76, 0x79, 0x18, 0x04, 0x20, 0x01, 0x28, 0x01, 0x52, 0x02, 0x76, 0x79, 0x22,
	0x70, 0x0a, 0x11, 0x67, 0x72, 0x53, 0x69, 0x6d, 0x5f, 0x52, 0x65, 0x70, 0x6c, 0x61, 0x63, 0x65,
	0x6d, 0x65, 0x6e, 0x74, 0x12, 0x2a, 0x0a, 0x04, 0x62, 0x61, 0x6c, 0x6c, 0x18, 0x01, 0x20, 0x01,
	0x28, 0x0b, 0x32, 0x16, 0x2e, 0x67, 0x72, 0x53, 0x69, 0x6d, 0x5f, 0x42, 0x61, 0x6c, 0x6c, 0x52,
	0x65, 0x70, 0x6c, 0x61, 0x63, 0x65, 0x6d, 0x65, 0x6e, 0x74, 0x52, 0x04, 0x62, 0x61, 0x6c, 0x6c,
	0x12, 0x2f, 0x0a, 0x06, 0x72, 0x6f, 0x62, 0x6f, 0x74, 0x73, 0x18, 0x02, 0x20, 0x03, 0x28, 0x0b,
	0x32, 0x17, 0x2e, 0x67, 0x72, 0x53, 0x69, 0x6d, 0x5f, 0x52, 0x6f, 0x62, 0x6f, 0x74, 0x52, 0x65,
	0x70, 0x6c, 0x61, 0x63, 0x65, 0x6d, 0x65, 0x6e, 0x74, 0x52, 0x06, 0x72, 0x6f, 0x62, 0x6f, 0x74,
	0x73, 0x42, 0x2d, 0x5a, 0x2b, 0x67, 0x69, 0x74, 0x68, 0x75, 0x62, 0x2e, 0x63, 0x6f, 0x6d, 0x2f,
	0x52, 0x69, 0x6f, 0x6e, 0x65, 0x2d, 0x53, 0x53, 0x4c, 0x2f, 0x52, 0x41, 0x43, 0x4f, 0x4f, 0x4e,
	0x2d, 0x50, 0x69, 0x2f, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2f, 0x70, 0x62, 0x5f, 0x67, 0x65, 0x6e,
}

var (
	file_grSim_Replacement_proto_rawDescOnce sync.Once
	file_grSim_Replacement_proto_rawDescData = file_grSim_Replacement_proto_rawDesc
)

func file_grSim_Replacement_proto_rawDescGZIP() []byte {
	file_grSim_Replacement_proto_rawDescOnce.Do(func() {
		file_grSim_Replacement_proto_rawDescData = protoimpl.X.CompressGZIP(file_grSim_Replacement_proto_rawDescData)
	})
	return file_grSim_Replacement_proto_rawDescData
}

var file_grSim_Replacement_proto_msgTypes = make([]protoimpl.MessageInfo, 3)
var file_grSim_Replacement_proto_goTypes = []interface{}{
	(*GrSim_RobotReplacement)(nil), // 0: grSim_RobotReplacement
	(*GrSim_BallReplacement)(nil),  // 1: grSim_BallReplacement
	(*GrSim_Replacement)(nil),      // 2: grSim_Replacement
}
var file_grSim_Replacement_proto_depIdxs = []int32{
	1, // 0: grSim_Replacement.ball:type_name -> grSim_BallReplacement
	0, // 1: grSim_Replacement.robots:type_name -> grSim_RobotReplacement
	2, // [2:2] is the sub-list for method output_type
	2, // [2:2] is the sub-list for method input_type
	2, // [2:2] is the sub-list for extension type_name
	2, // [2:2] is the sub-list for extension extendee
	0, // [0:2] is the sub-list for field type_name
}

func init() { file_grSim_Replacement_proto_init() }
func file_grSim_Replacement_proto_init() {
	if File_grSim_Replacement_proto != nil {
		return
	}
	if !protoimpl.UnsafeEnabled {
		file_grSim_Replacement_proto_msgTypes[0].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*GrSim_RobotReplacement); i {
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
		file_grSim_Replacement_proto_msgTypes[1].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*GrSim_BallReplacement); i {
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
		file_grSim_Replacement_proto_msgTypes[2].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*GrSim_Replacement); i {
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
			RawDescriptor: file_grSim_Replacement_proto_rawDesc,
			NumEnums:      0,
			NumMessages:   3,
			NumExtensions: 0,
			NumServices:   0,
		},
		GoTypes:           file_grSim_Replacement_proto_goTypes,
		DependencyIndexes: file_grSim_Replacement_proto_depIdxs,
		MessageInfos:      file_grSim_Replacement_proto_msgTypes,
	}.Build()
	File_grSim_Replacement_proto = out.File
	file_grSim_Replacement_proto_rawDesc = nil
	file_grSim_Replacement_proto_goTypes = nil
	file_grSim_Replacement_proto_depIdxs = nil
}
