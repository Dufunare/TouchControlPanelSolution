#pragma once

#include <string>

namespace touchpanel
{
    /// Pure virtual interface for communication strategies (Strategy Pattern).
    /// Concrete implementations (e.g. DobotDirectStrategy) are injected at runtime
    /// so the network-send thread stays decoupled from any specific protocol.
    class ICommunicationStrategy
    {
    public:
        virtual ~ICommunicationStrategy() = default;

        /// Establish connections to the target device / relay station.
        /// @param ip   Target IP address.
        /// @param port Primary control port (strategy may open additional ports).
        /// @returns true on success.
        virtual bool connect(const std::string& ip, unsigned short port) = 0;

        /// Disconnect from the target device.
        virtual void disconnect() = 0;

        /// Send a Cartesian coordinate to the target.
        /// Implementations are responsible for assembling the protocol-specific
        /// payload (ASCII command string, JSON, binary struct, etc.).
        /// @returns true if the data was sent successfully.
        virtual bool sendCoordinate(double x, double y, double z) = 0;

        /// @returns true when the strategy is connected and ready to send.
        virtual bool isConnected() const = 0;

        /// @returns A human-readable description of the last error, or empty.
        virtual std::string lastError() const = 0;
    };
} // namespace touchpanel
