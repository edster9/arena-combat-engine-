package main

import (
	"flag"
	"fmt"
	"os"
	"os/signal"
	"syscall"
)

func main() {
	port := flag.Int("port", 7777, "Server port")
	flag.Parse()

	fmt.Println("Car Wars Server v0.1")
	fmt.Println("====================")
	fmt.Printf("\nListening on port %d\n", *port)

	// TODO: Initialize subsystems
	// - net.Server
	// - lobby.Manager
	// - game.StateManager

	// Wait for shutdown signal
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	fmt.Println("\nShutting down...")
}
