// Code generated by protoc-gen-go. DO NOT EDIT.
// versions:
// 	protoc-gen-go v1.25.0-devel
// 	protoc        v3.12.4
// source: pi_to_mw.proto

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

type Robots_Status struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	RobotsStatus []*Robot_Status `protobuf:"bytes,1,rep,name=robots_status,json=robotsStatus" json:"robots_status,omitempty"`
}

func (x *Robots_Status) Reset() {
	*x = Robots_Status{}
	if protoimpl.UnsafeEnabled {
		mi := &file_pi_to_mw_proto_msgTypes[0]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *Robots_Status) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*Robots_Status) ProtoMessage() {}

func (x *Robots_Status) ProtoReflect() protoreflect.Message {
	mi := &file_pi_to_mw_proto_msgTypes[0]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use Robots_Status.ProtoReflect.Descriptor instead.
func (*Robots_Status) Descriptor() ([]byte, []int) {
	return file_pi_to_mw_proto_rawDescGZIP(), []int{0}
}

func (x *Robots_Status) GetRobotsStatus() []*Robot_Status {
	if x != nil {
		return x.RobotsStatus
	}
	return nil
}

type Robot_Status struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	RobotId        *uint32 `protobuf:"varint,1,req,name=robot_id,json=robotId" json:"robot_id,omitempty"`
	Infrared       *bool   `protobuf:"varint,2,req,name=infrared" json:"infrared,omitempty"`
	BatteryVoltage *uint32 `protobuf:"varint,3,req,name=battery_voltage,json=batteryVoltage" json:"battery_voltage,omitempty"`
	CapPower       *uint32 `protobuf:"varint,4,req,name=cap_power,json=capPower" json:"cap_power,omitempty"`
}

func (x *Robot_Status) Reset() {
	*x = Robot_Status{}
	if protoimpl.UnsafeEnabled {
		mi := &file_pi_to_mw_proto_msgTypes[1]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *Robot_Status) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*Robot_Status) ProtoMessage() {}

func (x *Robot_Status) ProtoReflect() protoreflect.Message {
	mi := &file_pi_to_mw_proto_msgTypes[1]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use Robot_Status.ProtoReflect.Descriptor instead.
func (*Robot_Status) Descriptor() ([]byte, []int) {
	return file_pi_to_mw_proto_rawDescGZIP(), []int{1}
}

func (x *Robot_Status) GetRobotId() uint32 {
	if x != nil && x.RobotId != nil {
		return *x.RobotId
	}
	return 0
}

func (x *Robot_Status) GetInfrared() bool {
	if x != nil && x.Infrared != nil {
		return *x.Infrared
	}
	return false
}

func (x *Robot_Status) GetBatteryVoltage() uint32 {
	if x != nil && x.BatteryVoltage != nil {
		return *x.BatteryVoltage
	}
	return 0
}

func (x *Robot_Status) GetCapPower() uint32 {
	if x != nil && x.CapPower != nil {
		return *x.CapPower
	}
	return 0
}

var File_pi_to_mw_proto protoreflect.FileDescriptor

var file_pi_to_mw_proto_rawDesc = []byte{
	0x0a, 0x0e, 0x70, 0x69, 0x5f, 0x74, 0x6f, 0x5f, 0x6d, 0x77, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f,
	0x22, 0x43, 0x0a, 0x0d, 0x52, 0x6f, 0x62, 0x6f, 0x74, 0x73, 0x5f, 0x53, 0x74, 0x61, 0x74, 0x75,
	0x73, 0x12, 0x32, 0x0a, 0x0d, 0x72, 0x6f, 0x62, 0x6f, 0x74, 0x73, 0x5f, 0x73, 0x74, 0x61, 0x74,
	0x75, 0x73, 0x18, 0x01, 0x20, 0x03, 0x28, 0x0b, 0x32, 0x0d, 0x2e, 0x52, 0x6f, 0x62, 0x6f, 0x74,
	0x5f, 0x53, 0x74, 0x61, 0x74, 0x75, 0x73, 0x52, 0x0c, 0x72, 0x6f, 0x62, 0x6f, 0x74, 0x73, 0x53,
	0x74, 0x61, 0x74, 0x75, 0x73, 0x22, 0x8b, 0x01, 0x0a, 0x0c, 0x52, 0x6f, 0x62, 0x6f, 0x74, 0x5f,
	0x53, 0x74, 0x61, 0x74, 0x75, 0x73, 0x12, 0x19, 0x0a, 0x08, 0x72, 0x6f, 0x62, 0x6f, 0x74, 0x5f,
	0x69, 0x64, 0x18, 0x01, 0x20, 0x02, 0x28, 0x0d, 0x52, 0x07, 0x72, 0x6f, 0x62, 0x6f, 0x74, 0x49,
	0x64, 0x12, 0x1a, 0x0a, 0x08, 0x69, 0x6e, 0x66, 0x72, 0x61, 0x72, 0x65, 0x64, 0x18, 0x02, 0x20,
	0x02, 0x28, 0x08, 0x52, 0x08, 0x69, 0x6e, 0x66, 0x72, 0x61, 0x72, 0x65, 0x64, 0x12, 0x27, 0x0a,
	0x0f, 0x62, 0x61, 0x74, 0x74, 0x65, 0x72, 0x79, 0x5f, 0x76, 0x6f, 0x6c, 0x74, 0x61, 0x67, 0x65,
	0x18, 0x03, 0x20, 0x02, 0x28, 0x0d, 0x52, 0x0e, 0x62, 0x61, 0x74, 0x74, 0x65, 0x72, 0x79, 0x56,
	0x6f, 0x6c, 0x74, 0x61, 0x67, 0x65, 0x12, 0x1b, 0x0a, 0x09, 0x63, 0x61, 0x70, 0x5f, 0x70, 0x6f,
	0x77, 0x65, 0x72, 0x18, 0x04, 0x20, 0x02, 0x28, 0x0d, 0x52, 0x08, 0x63, 0x61, 0x70, 0x50, 0x6f,
	0x77, 0x65, 0x72, 0x42, 0x2e, 0x5a, 0x2c, 0x67, 0x69, 0x74, 0x68, 0x75, 0x62, 0x2e, 0x63, 0x6f,
	0x6d, 0x2f, 0x52, 0x69, 0x6f, 0x6e, 0x65, 0x2f, 0x73, 0x73, 0x6c, 0x2d, 0x52, 0x41, 0x43, 0x4f,
	0x4f, 0x4e, 0x2d, 0x50, 0x69, 0x32, 0x2f, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2f, 0x70, 0x62, 0x5f,
	0x67, 0x65, 0x6e,
}

var (
	file_pi_to_mw_proto_rawDescOnce sync.Once
	file_pi_to_mw_proto_rawDescData = file_pi_to_mw_proto_rawDesc
)

func file_pi_to_mw_proto_rawDescGZIP() []byte {
	file_pi_to_mw_proto_rawDescOnce.Do(func() {
		file_pi_to_mw_proto_rawDescData = protoimpl.X.CompressGZIP(file_pi_to_mw_proto_rawDescData)
	})
	return file_pi_to_mw_proto_rawDescData
}

var file_pi_to_mw_proto_msgTypes = make([]protoimpl.MessageInfo, 2)
var file_pi_to_mw_proto_goTypes = []interface{}{
	(*Robots_Status)(nil), // 0: Robots_Status
	(*Robot_Status)(nil),  // 1: Robot_Status
}
var file_pi_to_mw_proto_depIdxs = []int32{
	1, // 0: Robots_Status.robots_status:type_name -> Robot_Status
	1, // [1:1] is the sub-list for method output_type
	1, // [1:1] is the sub-list for method input_type
	1, // [1:1] is the sub-list for extension type_name
	1, // [1:1] is the sub-list for extension extendee
	0, // [0:1] is the sub-list for field type_name
}

func init() { file_pi_to_mw_proto_init() }
func file_pi_to_mw_proto_init() {
	if File_pi_to_mw_proto != nil {
		return
	}
	if !protoimpl.UnsafeEnabled {
		file_pi_to_mw_proto_msgTypes[0].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*Robots_Status); i {
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
		file_pi_to_mw_proto_msgTypes[1].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*Robot_Status); i {
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
			RawDescriptor: file_pi_to_mw_proto_rawDesc,
			NumEnums:      0,
			NumMessages:   2,
			NumExtensions: 0,
			NumServices:   0,
		},
		GoTypes:           file_pi_to_mw_proto_goTypes,
		DependencyIndexes: file_pi_to_mw_proto_depIdxs,
		MessageInfos:      file_pi_to_mw_proto_msgTypes,
	}.Build()
	File_pi_to_mw_proto = out.File
	file_pi_to_mw_proto_rawDesc = nil
	file_pi_to_mw_proto_goTypes = nil
	file_pi_to_mw_proto_depIdxs = nil
}
