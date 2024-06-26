# Module that contains OSIDataContainer class to handle and manage OSI traces.
# Modified from OSI Validation https://github.com/OpenSimulationInterface/osi-validation
# Copyright 2023 BMW AG
# SPDX-License-Identifier: MPL-2.0

from collections import deque
import time
from multiprocessing import Manager
import lzma
import struct

from osi3.osi_sensorview_pb2 import SensorView
from osi3.osi_groundtruth_pb2 import GroundTruth
from osi3.osi_sensordata_pb2 import SensorData
import warnings

from lib.linked_proto_field import LinkedProtoField

warnings.simplefilter("default")

SEPARATOR = b"$$__$$"
SEPARATOR_LENGTH = len(SEPARATOR)


def get_size_from_file_stream(file_object):
    """
    Return a file size from a file stream given in parameters
    """
    current_position = file_object.tell()
    file_object.seek(0, 2)
    size = file_object.tell()
    file_object.seek(current_position)
    return size


MESSAGES_TYPE = {
    "SensorView": SensorView,
    "GroundTruth": GroundTruth,
    "SensorData": SensorData,
}


class OSITrace:
    """This class wrap OSI data. It can import and decode OSI traces."""

    def __init__(self, buffer_size, show_progress=True, type_name="SensorView"):
        self.trace_file = None
        self.message_offsets = None
        self.buffer_size = buffer_size
        self._int_length = len(struct.pack("<L", 0))
        self.type_name = type_name
        self.manager = Manager()
        self.message_cache = self.manager.dict()
        self.timestep_count = 0
        self.show_progress = show_progress
        self.retrieved_trace_size = 0

    # Open and Read text file
    def from_file(self, path, type_name="SensorView", max_index=-1):
        """Import a trace from a file"""
        self.path = path
        if self.path.lower().endswith((".lzma", ".xz")):
            self.trace_file = lzma.open(self.path, "rb")
        else:
            self.trace_file = open(self.path, "rb")

        self.type_name = type_name

        if self.path.lower().endswith((".txt.lzma", ".txt.xz", ".txt")):
            warnings.warn(
                "The separated trace files will be completely removed in the near future. Please convert them to *.osi files with the converter in the main OSI repository.",
                PendingDeprecationWarning,
            )
            self.timestep_count = self.retrieve_message_offsets(max_index)
        else:
            self.timestep_count = self.retrieve_message()

    def retrieve_message(self):
        """
        Retrieve the offsets of all the messages of the osi trace and store them
        in the `message_offsets` attribute of the object

        It returns the number of discovered timesteps
        """
        trace_size = get_size_from_file_stream(self.trace_file)

        progress_bar = None

        eof = False

        if self.show_progress:
            start_time = time.time()

        self.trace_file.seek(0)

        self.message_offsets = [0]
        message_offset = 0
        message_length = 0
        counter = 0  # Counter is needed to enable correct buffer parsing of serialized messages

        # Check if user decided to use buffer
        if self.buffer_size != 0 and type(self.buffer_size) == int:

            # Run while the end of file is not reached
            while not eof and message_offset < trace_size:
                serialized_message = self.trace_file.read(self.buffer_size)
                self.trace_file.seek(self.message_offsets[-1])

                while not eof:

                    # Unpack the message size relative to the current buffer
                    message_length = struct.unpack(
                        "<L",
                        serialized_message[
                        message_offset
                        - counter * self.buffer_size: self._int_length
                                                      + message_offset
                                                      - counter * self.buffer_size
                        ],
                    )[0]

                    # Get the message offset of the next message
                    message_offset += message_length + self._int_length
                    self.message_offsets.append(message_offset)
                    self.update_bar(progress_bar, message_offset)
                    self.trace_file.seek(message_offset)
                    eof = self.trace_file.tell() > self.buffer_size * (counter + 1)

                    # Check if reached end of file
                    if self.trace_file.tell() == trace_size:
                        self.retrieved_trace_size = self.message_offsets[-1]
                        self.message_offsets.pop()  # Remove the last element since after that there is no message coming
                        break

                while eof:

                    # Counter increment and cursor placement update. The cursor is set absolute in the file.
                    if message_offset >= len(serialized_message):
                        self.update_bar(progress_bar, message_offset)
                        counter += 1
                        self.trace_file.seek(counter * self.buffer_size)
                        eof = False

        else:
            serialized_message = self.trace_file.read()
            while message_offset < trace_size:
                message_length = struct.unpack(
                    "<L",
                    serialized_message[
                    message_offset: self._int_length + message_offset
                    ],
                )[0]
                message_offset += message_length + self._int_length
                self.message_offsets.append(message_offset)
                self.update_bar(progress_bar, message_offset)

            self.retrieved_trace_size = self.message_offsets[-1]
            self.message_offsets.pop()

        return len(self.message_offsets)

    def retrieve_message_offsets(self, max_index):
        """
        Retrieve the offsets of all the messages of the txt trace and store them
        in the `message_offsets` attribute of the object

        It returns the number of discovered timesteps
        """
        trace_size = get_size_from_file_stream(self.trace_file)

        if max_index == -1:
            max_index = float("inf")

        # For $$__$$ separated trace files the buffersize needs to be greater than zero
        if self.buffer_size == 0:
            self.buffer_size = 1000000  # Make it backwards compatible

        progress_bar = None

        buffer_deque = deque(maxlen=2)

        self.message_offsets = [0]
        eof = False

        if self.show_progress:
            start_time = time.time()

        self.trace_file.seek(0)

        while not eof and len(self.message_offsets) <= max_index:
            found = -1  # SEP offset in buffer
            buffer_deque.clear()

            while found == -1 and not eof:
                new_read = self.trace_file.read(self.buffer_size)
                buffer_deque.append(new_read)
                buffer = b"".join(buffer_deque)
                found = buffer.find(SEPARATOR)
                eof = len(new_read) != self.buffer_size

            buffer_offset = self.trace_file.tell() - len(buffer)
            message_offset = found + buffer_offset + SEPARATOR_LENGTH
            self.message_offsets.append(message_offset)
            self.update_bar(progress_bar, message_offset)
            self.trace_file.seek(message_offset)

            while eof and found != -1:
                buffer = buffer[found + SEPARATOR_LENGTH:]
                found = buffer.find(SEPARATOR)

                buffer_offset = trace_size - len(buffer)

                message_offset = found + buffer_offset + SEPARATOR_LENGTH

                if message_offset >= trace_size:
                    break
                self.message_offsets.append(message_offset)

                self.update_bar(progress_bar, message_offset)

        if eof:
            self.retrieved_trace_size = trace_size
        else:
            self.retrieved_trace_size = self.message_offsets[-1]
            self.message_offsets.pop()

        if self.show_progress:
            progress_bar.finish()
            print(
                len(self.message_offsets),
                "messages has been discovered in",
                time.time() - start_time,
                "s",
            )

        return len(self.message_offsets)

    def get_message_by_index(self, index):
        """
        Get a message by its index. Try first to get it from the cache made
        by the method ``cache_messages_in_index_range``.
        """
        message = self.message_cache.get(index, None)

        if message is not None:
            return message

        message = next(self.get_messages_in_index_range(index, index + 1))
        return LinkedProtoField(message, name=self.type_name)

    def get_messages_in_index_range(self, begin, end):
        """
        Yield an iterator over messages of indexes between begin and end included.
        """
        progress_bar = None

        self.trace_file.seek(self.message_offsets[begin])
        abs_first_offset = self.message_offsets[begin]
        abs_last_offset = (
            self.message_offsets[end]
            if end < len(self.message_offsets)
            else self.retrieved_trace_size
        )

        rel_message_offsets = [
            abs_message_offset - abs_first_offset
            for abs_message_offset in self.message_offsets[begin:end]
        ]

        if self.path.lower().endswith((".txt.lzma", ".txt.xz", ".txt")):
            message_sequence_len = abs_last_offset - abs_first_offset - SEPARATOR_LENGTH
            serialized_messages_extract = self.trace_file.read(message_sequence_len)

            for rel_index, rel_message_offset in enumerate(rel_message_offsets):
                rel_begin = rel_message_offset
                rel_end = (
                    rel_message_offsets[rel_index + 1] - SEPARATOR_LENGTH
                    if rel_index + 1 < len(rel_message_offsets)
                    else message_sequence_len
                )

                message = MESSAGES_TYPE[self.type_name]()
                serialized_message = serialized_messages_extract[rel_begin:rel_end]
                message.ParseFromString(serialized_message)
                self.update_bar(progress_bar, rel_index)
                yield LinkedProtoField(message, name=self.type_name)

        elif self.path.lower().endswith((".osi.lzma", ".osi.xz", ".osi")):
            message_sequence_len = abs_last_offset - abs_first_offset
            serialized_messages_extract = self.trace_file.read(message_sequence_len)

            for rel_index, rel_message_offset in enumerate(rel_message_offsets):
                rel_begin = rel_message_offset + self._int_length
                rel_end = (
                    rel_message_offsets[rel_index + 1] - self._int_length
                    if rel_index + 1 < len(rel_message_offsets)
                    else message_sequence_len
                )

                message = MESSAGES_TYPE[self.type_name]()
                serialized_message = serialized_messages_extract[rel_begin:rel_end]
                message.ParseFromString(serialized_message)
                self.update_bar(progress_bar, rel_index)
                yield LinkedProtoField(message, name=self.type_name)

        else:
            raise Exception(
                f"The defined file format {self.path.split('/')[-1]} does not exist."
            )

        if self.show_progress:
            self.update_bar(progress_bar, progress_bar.max)
            progress_bar.finish()

    def cache_messages_in_index_range(self, begin, end):
        """
        Put all messages from index begin to index end in the cache. Then the
        method ``get_message_by_index`` can access to it in a faster way.

        Using this method again clear the last cache and replace it with a new
        one.
        """
        if self.show_progress:
            print("\nCaching ...")
        self.message_cache = self.manager.dict(
            {
                index + begin: message
                for index, message in enumerate(
                self.get_messages_in_index_range(begin, end)
            )
            }
        )
        if self.show_progress:
            print("Caching done!")

    def update_bar(self, progress_bar, new_index):
        if self.show_progress and progress_bar is not None:
            progress_bar.index = new_index
            progress_bar.update()
