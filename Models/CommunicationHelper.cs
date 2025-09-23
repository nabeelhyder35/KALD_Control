using System;
using System.Linq;

namespace KALD_Control.Models
{
    /// <summary>
    /// Provides helper methods for communication protocol handling.
    /// </summary>
    public static class CommunicationHelper
    {
        private static readonly Action<string> Log = message => System.Diagnostics.Debug.WriteLine($"[CommunicationHelper] {message}");

        /// <summary>
        /// Creates a packet according to the specified protocol structure.
        /// </summary>
        /// <param name="command">The command byte.</param>
        /// <param name="data">The data payload.</param>
        /// <returns>The complete packet as a byte array.</returns>
        public static byte[] CreatePacket(byte command, byte[] data = null)
        {
            data = data ?? Array.Empty<byte>();
            int dataLength = data.Length;
            int packetLength = 1 + 2 + 1 + dataLength + 1 + 1; // STX + LEN(2) + CMD + DATA + CHK + ETX

            // Length field represents only the data length
            ushort lengthField = (ushort)dataLength;

            byte[] packet = new byte[packetLength];
            int index = 0;

            // STX
            packet[index++] = ProtocolConstants.STX;

            // Length (2 bytes, big-endian)
            packet[index++] = (byte)(lengthField >> 8);
            packet[index++] = (byte)(lengthField & 0xFF);

            // Command
            packet[index++] = command;

            // Data
            if (dataLength > 0)
            {
                Array.Copy(data, 0, packet, index, dataLength);
                index += dataLength;
            }

            // Calculate checksum (0 - sum of command and data bytes)
            byte sum = command;
            for (int i = 0; i < dataLength; i++)
            {
                sum += data[i];
            }
            packet[index++] = (byte)((0 - sum) & 0xFF);

            // ETX
            packet[index] = ProtocolConstants.ETX;

            Log($"Created packet: {BitConverter.ToString(packet).Replace("-", " ")}");
            return packet;
        }

        /// <summary>
        /// Validates a received packet according to the protocol structure.
        /// </summary>
        /// <param name="packet">The received packet.</param>
        /// <returns>The validation result.</returns>
        public static PacketValidationResult ValidatePacket(byte[] packet)
        {
            if (packet == null || packet.Length < ProtocolConstants.MIN_PACKET_LENGTH)
            {
                Log($"Validation failed: Insufficient data, length={packet?.Length ?? 0}");
                return PacketValidationResult.InsufficientData;
            }

            // Check STX
            if (packet[0] != ProtocolConstants.STX)
            {
                Log($"Validation failed: Invalid STX, got 0x{packet[0]:X2}");
                return PacketValidationResult.InvalidSTX;
            }

            // Extract length (big-endian)
            ushort lengthField = (ushort)((packet[1] << 8) | packet[2]);

            // Verify packet length
            int expectedPacketLength = 1 + 2 + 1 + lengthField + 1 + 1; // STX + LEN(2) + CMD + DATA + CHK + ETX
            if (packet.Length < expectedPacketLength)
            {
                Log($"Validation failed: Insufficient data, expected {expectedPacketLength}, got {packet.Length}");
                return PacketValidationResult.InsufficientData;
            }

            // Check ETX
            if (packet[expectedPacketLength - 1] != ProtocolConstants.ETX)
            {
                Log($"Validation failed: Invalid ETX, got 0x{packet[expectedPacketLength - 1]:X2}");
                return PacketValidationResult.InvalidETX;
            }

            // Verify checksum
            int dataEnd = expectedPacketLength - 2; // Position before checksum
            byte sum = packet[3]; // Command
            for (int i = 4; i < 4 + lengthField; i++)
            {
                sum += packet[i];
            }
            byte calculatedChecksum = (byte)((0 - sum) & 0xFF);
            byte receivedChecksum = packet[expectedPacketLength - 2];

            if (calculatedChecksum != receivedChecksum)
            {
                Log($"Validation failed: Checksum mismatch, expected 0x{calculatedChecksum:X2}, got 0x{receivedChecksum:X2}");
                return PacketValidationResult.ChecksumMismatch;
            }

            Log($"Packet validated: Length={lengthField}, Checksum=0x{receivedChecksum:X2}");
            return PacketValidationResult.Valid;
        }

        /// <summary>
        /// Extracts command and data from a valid packet.
        /// </summary>
        /// <param name="packet">The validated packet.</param>
        /// <param name="command">The extracted command.</param>
        /// <param name="data">The extracted data.</param>
        /// <returns>True if extraction was successful.</returns>
        public static bool ExtractPacketData(byte[] packet, out byte command, out byte[] data)
        {
            command = 0;
            data = null;

            if (ValidatePacket(packet) != PacketValidationResult.Valid)
            {
                Log("Extraction failed: Invalid packet");
                return false;
            }

            // Extract length (big-endian)
            ushort lengthField = (ushort)((packet[1] << 8) | packet[2]);

            // Command is at position 3
            command = packet[3];

            // Data length is lengthField
            int dataLength = lengthField;

            if (dataLength > 0)
            {
                data = new byte[dataLength];
                Array.Copy(packet, 4, data, 0, dataLength);
            }
            else
            {
                data = Array.Empty<byte>();
            }

            Log($"Extracted: Command=0x{command:X2}, Data=[{BitConverter.ToString(data).Replace("-", " ")}]");
            return true;
        }
    }
}