#!/usr/bin/env python3
"""
RFID Simulator Node for SmartCart Simulation.

Simulates an RFID item scanner and running billing cart:
- Looks up scanned RFID tags against product database
- Prevents duplicate product scans
- Computes and displays running total
- Publishes scanned item tags to `/rfid/item` (std_msgs/msg/String)
- Supports interactive terminal commands: tag IDs, clear, list, help, quit
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Predefined RFID Product Catalog (Authoritative Specification §5.12)
PRODUCTS = {
    "RFID001": ("Milk", 40.0),
    "RFID002": ("Bread", 35.0),
    "RFID003": ("Apple", 20.0),
    "RFID004": ("Biscuit", 30.0),
}


class RFIDSimulator(Node):
    """Interactive RFID item scanner node with duplicate protection and billing."""

    def __init__(self):
        super().__init__('rfid_simulator')

        # Publisher for detected RFID items
        self.item_pub = self.create_publisher(String, '/rfid/item', 10)

        # Cart state: dict mapping tag_id -> (product_name, price)
        self.cart = {}
        self.running = True

        self.get_logger().info('RFID Simulator node initialized.')
        self.print_welcome()

        # Start interactive CLI in background daemon thread
        self.input_thread = threading.Thread(target=self.cli_loop, daemon=True)
        self.input_thread.start()

    def print_welcome(self):
        """Display startup information and available products."""
        print("\n==================================================", flush=True)
        print("           SmartCart RFID Item Simulator          ", flush=True)
        print("==================================================", flush=True)
        print("Available Product Tags:", flush=True)
        for tag, (name, price) in PRODUCTS.items():
            print(f"  [{tag}] {name:<10} - ₹{price:.2f}", flush=True)
        print("--------------------------------------------------", flush=True)
        print("Commands:", flush=True)
        print("  <TAG_ID>     : Scan an RFID tag (e.g. RFID001)", flush=True)
        print("  list / total : View current cart and total bill", flush=True)
        print("  clear / c    : Empty the shopping cart", flush=True)
        print("  help         : Show this help menu", flush=True)
        print("  quit / exit  : Exit the RFID simulator", flush=True)
        print("==================================================\n", flush=True)

    def print_cart(self):
        """Print current shopping cart and total bill."""
        print("\n--- Current Cart ---", flush=True)
        if not self.cart:
            print("  (Cart is empty)", flush=True)
            print("--------------------", flush=True)
            print("Total: ₹0.00\n", flush=True)
            return

        total = 0.0
        for tag, (name, price) in self.cart.items():
            print(f"  {name:<12} ₹{price:.2f}", flush=True)
            total += price
        print("--------------------", flush=True)
        print(f"Total:       ₹{total:.2f}\n", flush=True)

    def process_tag(self, raw_input_str: str):
        """Process a scanned RFID tag or user command."""
        tag = raw_input_str.strip()
        if not tag:
            return

        tag_upper = tag.upper()

        # Handle commands
        if tag_upper in ("CLEAR", "C"):
            self.cart.clear()
            print("Cart cleared. Total: ₹0.00\n", flush=True)
            return

        if tag_upper in ("LIST", "TOTAL", "BILL"):
            self.print_cart()
            return

        if tag_upper in ("HELP", "H", "?"):
            self.print_welcome()
            return

        if tag_upper in ("QUIT", "EXIT", "Q"):
            print("Exiting RFID Simulator...", flush=True)
            self.running = False
            return

        # Check against product catalog
        if tag_upper in PRODUCTS:
            name, price = PRODUCTS[tag_upper]
            if tag_upper in self.cart:
                print(f"Already in cart: {name} (Tag: {tag_upper})", flush=True)
                total = sum(p for _, p in self.cart.values())
                print(f"Current Total: ₹{total:.2f}\n", flush=True)
            else:
                self.cart[tag_upper] = (name, price)
                print(f"\nRFID detected: {tag_upper}", flush=True)
                print(f"Product: {name}", flush=True)
                print(f"Price: ₹{price:.2f}", flush=True)

                total = sum(p for _, p in self.cart.values())
                print(f"Current Total: ₹{total:.2f}\n", flush=True)

                # Publish tag to ROS topic
                msg = String()
                msg.data = tag_upper
                self.item_pub.publish(msg)
                self.get_logger().info(f"Published item '{tag_upper}' ({name}) to /rfid/item")
        else:
            print(f"Unknown RFID tag: {tag}\n", flush=True)

    def cli_loop(self):
        """Read lines from standard input interactively."""
        while self.running and rclpy.ok():
            try:
                line = sys.stdin.readline()
                if not line:
                    self.running = False
                    break
                self.process_tag(line)
            except Exception as e:
                self.get_logger().error(f"Error reading RFID input: {e}")
                self.running = False
                break


def main(args=None):
    rclpy.init(args=args)
    node = RFIDSimulator()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
