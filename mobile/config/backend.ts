/**
 * Backend Configuration
 * 
 * IMPORTANT: For Expo Go on iPad/iPhone, make sure:
 * 1. Your iPad and computer are on the same Wi-Fi network
 * 2. The backend server is running on your computer
 * 3. The IP address below matches your computer's local IP address
 * 
 * To find your computer's IP address:
 * - Mac: System Settings > Network > Wi-Fi > Details (or run `ifconfig | grep "inet "` in Terminal)
 * - Windows: Settings > Network & Internet > Wi-Fi > Properties (or run `ipconfig` in CMD)
 * - Linux: Run `ip addr show` or `hostname -I`
 * 
 * Make sure to use your computer's IP address, not localhost or 127.0.0.1
 */

// Change this to your computer's local IP address
// Example: "http://192.168.1.100:5000"
export const BACKEND_URL = "http://192.168.254.255:5000";

// Alternative: Use environment variable if available (for production builds)
// export const BACKEND_URL = process.env.EXPO_PUBLIC_BACKEND_URL || "http://192.168.0.217:5000";

