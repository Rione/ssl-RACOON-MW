package main

import (
	"fmt"
	"log"

	"net"
	"os"
	"os/signal"

	"errors"
	"io"
	"net/http"
	"strconv"

	"time"

	imagepb "github.com/Rione-SSL/RACOON-MW/proto/pb_gen"

	"gocv.io/x/gocv"
	"google.golang.org/grpc"
	"google.golang.org/grpc/reflection"
)

var (
	window *gocv.Window
	webcam *gocv.VideoCapture
	frame  = make([][]byte, 10)
)

type ImageServer struct {
	imagepb.UnimplementedImageServiceServer
}

func NewPictureServer() *ImageServer {
	return &ImageServer{}
}

func (s *ImageServer) ClientStream(stream imagepb.ImageService_ClientStreamServer) error {
	for {
		res, err := stream.Recv()
		if errors.Is(err, io.EOF) {
			fmt.Println("All responses have already been received.")
			break
		}
		if err != nil {
			fmt.Println("Error receiving stream:", err)
			break
		}

		if res == nil {
			fmt.Println("Received nil response")
			break
		}

		fmt.Println(res.Image)
		imgData, err := gocv.IMDecode(res.Image, gocv.IMReadColor)
		if err != nil {
			log.Printf("failed to decode image: %v", err)
			continue
		}
		if imgData.Empty() {
			log.Println("received empty image")
			continue
		}
		mutex.Lock()
		frame[int(res.Id)-1] = res.Image
		mutex.Unlock()
	}
	return nil
}

func helloHandler(w http.ResponseWriter, r *http.Request) {
	hello := []byte("Hello World!!!")
	_, err := w.Write(hello)
	if err != nil {
		log.Fatal(err)
	}
}

func videoHandler(w http.ResponseWriter, r *http.Request) {
	id := r.URL.Query().Get("id") // クエリパラメータでidを取得
	idInt, err := strconv.Atoi(id)
	if err != nil {
		log.Printf("invalid id: %v", err)
		return
	}
	if id == "" {
		id = "1" // idが指定されていない場合はデフォルトで1を使用
	}
	w.Header().Set("Content-Type", "multipart/x-mixed-replace; boundary=frame")
	data := ""
	for {
		data = "--frame\r\nContent-Type: image/jpg\r\n\r\n"
		w.Write([]byte(data))
		mutex.Lock()
		w.Write(frame[idInt-1]) // idに対応するフレームを送信
		mutex.Unlock()
		w.Write([]byte("\r\n\r\n"))
		time.Sleep(33 * time.Millisecond) // 約30fpsのタイミングに設定
	}
}

func htmlHandler(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/html")
	html := `
	<!DOCTYPE html>
	<html lang="en">
	<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Multiple Robot Video Streams</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
            background-color: #f4f4f4;
            flex-wrap: wrap; /* 複数のiframeを横並び、または次の行に表示 */
        }
        iframe {
            width: 400px;
            height: 300px;
            margin: 10px;
            border: none;
            border-radius: 10px; /* 角を丸くする */
            background-color: #333;
        }
    </style>
	</head>
	<body>
		<iframe src="http://localhost:8080/video?id=1" title="Robot Video Stream 1"></iframe>
		<iframe src="http://localhost:8080/video?id=2" title="Robot Video Stream 2"></iframe>
		<iframe src="http://localhost:8080/video?id=3" title="Robot Video Stream 3"></iframe>
		<iframe src="http://localhost:8080/video?id=4" title="Robot Video Stream 4"></iframe>
	</body>
	</html>
	`
	w.Write([]byte(html))
}

func Streaming(chstreaming chan bool) {
	port := "8081"

	listener, err := net.Listen("tcp", fmt.Sprintf(":%v", port))
	if err != nil {
		log.Fatalf("failed to listen: %v", err)
	}

	window = gocv.NewWindow("Client Stream")
	defer window.Close() // プログラム終了時にウィンドウを閉じる

	s := grpc.NewServer()
	imagepb.RegisterImageServiceServer(s, NewPictureServer())
	reflection.Register(s)

	go func() {
		log.Printf("start gRPC server address: %v", port)
		s.Serve(listener)
	}()

	go func() {
		http.HandleFunc("/hello", helloHandler)
		http.HandleFunc("/video", videoHandler)
		http.HandleFunc("/", htmlHandler)
		fmt.Println("start http server")
		http.ListenAndServe(":8080", nil)
	}()

	quit := make(chan os.Signal, 1)
	signal.Notify(quit, os.Interrupt)
	<-quit
	log.Println("stopping gRPC server...")
	s.GracefulStop()
}
