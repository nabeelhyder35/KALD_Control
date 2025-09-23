using System;
using KALD_Control.Models;

namespace KALD_Control.Services
{
    public enum PacketValidationResult
    {
        Valid,
        InvalidStart,
        InvalidEnd,
        InvalidLength,
        InvalidChecksum
    }

    public static class CommunicationHelper
    {
        public static byte[] CreatePacket(byte command, byte[] data)
        {
            if (data == null) data = new byte[0];
            int dataLength = data.Length;
            byte[] packet = new byte[ProtocolConstants.HEADER_LENGTH + dataLength + 1];
            packet[0] = ProtocolConstants.STX;
            packet[1] = (byte)(dataLength >> 8);
            packet[2] = (byte)(dataLength & 0xFF);
            packet[3] = command;
            for (int i = 0; i < dataLength; i++)
            {
                packet[4 + i] = data[i];
            }
            packet[packet.Length - 2] = CalculateChecksum(packet, 1, packet.Length - 3);
            packet[packet.Length - 1] = ProtocolConstants.ETX;
            return packet;
        }

        public static PacketValidationResult ValidatePacket(byte[] packet)
        {
            if (packet == null || packet.Length < ProtocolConstants.MIN_PACKET_LENGTH)
                return PacketValidationResult.InvalidLength;

            if (packet[0] != ProtocolConstants.STX)
                return PacketValidationResult.InvalidStart;

            if (packet[packet.Length - 1] != ProtocolConstants.ETX)
                return PacketValidationResult.InvalidEnd;

            int dataLength = (packet[1] << 8) | packet[2];
            if (packet.Length != ProtocolConstants.HEADER_LENGTH + dataLength + 1)
                return PacketValidationResult.InvalidLength;

            byte checksum = CalculateChecksum(packet, 1, packet.Length - 3);
            if (checksum != packet[packet.Length - 2])
                return PacketValidationResult.InvalidChecksum;

            return PacketValidationResult.Valid;
        }

        public static bool ExtractPacketData(byte[] packet, out byte command, out byte[] data)
        {
            command = 0;
            data = null;

            if (packet == null || packet.Length < ProtocolConstants.MIN_PACKET_LENGTH)
                return false;

            int dataLength = (packet[1] << 8) | packet[2];
            if (packet.Length != ProtocolConstants.HEADER_LENGTH + dataLength + 1)
                return false;

            command = packet[3];
            data = new byte[dataLength];
            Array.Copy(packet, 4, data, 0, dataLength);
            return true;
        }

        private static byte CalculateChecksum(byte[] data, int start, int length)
        {
            byte checksum = 0;
            for (int i = start; i < start + length; i++)
            {
                checksum ^= data[i];
            }
            return checksum;
        }
    }
}