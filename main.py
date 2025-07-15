import os
import sys

import cv2
import requests
from PyQt5.QtCore import QPoint, QRect, QSize, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (QApplication, QDialog, QFormLayout, QGroupBox,
                             QLabel, QLineEdit, QListWidget, QListWidgetItem,
                             QMainWindow, QPushButton, QTableWidget,
                             QTableWidgetItem, QVBoxLayout, QWidget)
from PyQt5.uic import loadUi


class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage, int)

    def __init__(self, rtsp_url, camera_index):
        super().__init__()
        self._run_flag = True
        self.rtsp_url = rtsp_url
        self.camera_index = camera_index

    def run(self):
        try:
            print(f"VideoThread {self.camera_index}: Attempting to open video stream: {self.rtsp_url})")
            cap = cv2.VideoCapture(self.rtsp_url)
            if not cap.isOpened():
                print(f"VideoThread {self.camera_index}: Error: Could not open video stream from {self.rtsp_url})")
                self._run_flag = False
                return
            print(f"VideoThread {self.camera_index}: Video stream opened successfully.")

            while self._run_flag:
                ret, frame = cap.read()
                if ret:
                    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgb_image.shape
                    bytes_per_line = ch * w
                    convert_to_qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                    p = convert_to_qt_format.scaled(640, 480, Qt.KeepAspectRatio)
                    self.change_pixmap_signal.emit(p, self.camera_index)
                else:
                    print(f"VideoThread {self.camera_index}: Error: Could not read frame. Breaking loop.")
                    break
            print(f"VideoThread {self.camera_index}: Releasing video capture.")
            cap.release()
        except Exception as e:
            import traceback
            with open("error.txt", "a") as f:
                f.write(f"\n--- Crash in VideoThread for {self.rtsp_url} ---\n")
                f.write(f"Error: {e}\n")
                f.write(traceback.format_exc())
                f.write("------------------------------------\n")
            self._run_flag = False # Stop the thread on error

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()


import ipaddress
import socket


class CameraManagerDialog(QDialog):
    def __init__(self, parent=None, initial_configs=None):
        super().__init__(parent)
        ui_file_path = os.path.join(os.path.dirname(__file__), 'camera_manager.ui')
        loadUi(ui_file_path, self)

        self.camera_configs = initial_configs if initial_configs is not None else []
        self.found_cameras = [] # To store discovered cameras

        self.init_camera_table()
        self.populate_table_with_configs()

        self.discoverButton.clicked.connect(self.discover_cameras)
        self.buttonBox.accepted.connect(self.accept_selection)
        self.buttonBox.rejected.connect(self.reject)

    def init_camera_table(self):
        headers = ["Select", "IP Address", "ID", "Password"]
        self.cameraTableWidget.setHorizontalHeaderLabels(headers)
        self.cameraTableWidget.setColumnCount(len(headers))
        self.cameraTableWidget.setRowCount(0) # Start with no rows

        # Set column widths
        self.cameraTableWidget.setColumnWidth(0, 60) # Select checkbox
        self.cameraTableWidget.setColumnWidth(1, 150) # IP Address
        self.cameraTableWidget.setColumnWidth(2, 100) # ID
        self.cameraTableWidget.setColumnWidth(3, 100) # Password

        self.cameraTableWidget.itemChanged.connect(self.handle_item_changed)

    def populate_table_with_configs(self):
        for config in self.camera_configs:
            self.add_camera_to_table(config['ip'], config['id'], config['password'], checked=True)

    def add_camera_to_table(self, ip, _id, pw, checked=False):
        row_position = self.cameraTableWidget.rowCount()
        self.cameraTableWidget.insertRow(row_position)

        # Select Checkbox
        checkbox_item = QTableWidgetItem()
        checkbox_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
        checkbox_item.setCheckState(Qt.Checked if checked else Qt.Unchecked)
        self.cameraTableWidget.setItem(row_position, 0, checkbox_item)

        # IP Address (read-only for discovered cameras)
        ip_item = QTableWidgetItem(ip)
        ip_item.setFlags(ip_item.flags() & ~Qt.ItemIsEditable)
        self.cameraTableWidget.setItem(row_position, 1, ip_item)

        # ID
        id_lineEdit = QLineEdit(_id)
        self.cameraTableWidget.setCellWidget(row_position, 2, id_lineEdit)

        # Password
        pw_lineEdit = QLineEdit(pw)
        pw_lineEdit.setEchoMode(QLineEdit.Password)
        self.cameraTableWidget.setCellWidget(row_position, 3, pw_lineEdit)

    def handle_item_changed(self, item):
        if item.column() == 0: # Only interested in changes to the checkbox column
            checked_count = 0
            for row in range(self.cameraTableWidget.rowCount()):
                checkbox_item = self.cameraTableWidget.item(row, 0)
                if checkbox_item and checkbox_item.checkState() == Qt.Checked:
                    checked_count += 1

            if checked_count > 4:
                item.setCheckState(Qt.Unchecked) # Uncheck if more than 4 are selected
                print("You can select a maximum of 4 cameras.")

    def discover_cameras(self):
        self.cameraTableWidget.setRowCount(0) # Clear existing entries
        self.found_cameras.clear()

        # Get local IP address and subnet mask
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80)) # Connect to a public IP to get local IP
            local_ip = s.getsockname()[0]
            s.close()

            # Assuming a /24 subnet mask for simplicity (e.g., 192.168.0.1/24)
            network = ipaddress.ip_network(f'{local_ip}/24', strict=False)
            print(f"Scanning network: {network}")

            # Create a thread for scanning to avoid freezing the UI
            self.scan_thread = CameraScanThread(network)
            self.scan_thread.camera_found_signal.connect(self.add_discovered_camera)
            self.scan_thread.scan_finished_signal.connect(lambda: print("Camera discovery finished."))
            self.scan_thread.start()

        except Exception as e:
            print(f"Error getting local IP or setting up network scan: {e}")

    def add_discovered_camera(self, ip_address):
        # Check if camera already in table
        for row in range(self.cameraTableWidget.rowCount()):
            if self.cameraTableWidget.item(row, 1).text() == ip_address:
                return # Already exists

        self.add_camera_to_table(ip_address, "admin", "admin")

    def accept_selection(self):
        selected_configs = []
        for row in range(self.cameraTableWidget.rowCount()):
            checkbox_item = self.cameraTableWidget.item(row, 0)
            if checkbox_item and checkbox_item.checkState() == Qt.Checked:
                ip = self.cameraTableWidget.item(row, 1).text()
                _id = self.cameraTableWidget.cellWidget(row, 2).text()
                pw = self.cameraTableWidget.cellWidget(row, 3).text()
                selected_configs.append({'ip': ip, 'id': _id, 'password': pw})
        self.camera_configs = selected_configs
        self.accept()

    def get_camera_configs(self):
        return self.camera_configs


class CameraScanThread(QThread):
    camera_found_signal = pyqtSignal(str)
    scan_finished_signal = pyqtSignal()

    def __init__(self, network):
        super().__init__()
        self.network = network
        self._run_flag = True

    def run(self):
        target_port = 60110
        for ip_int in self.network.hosts():
            if not self._run_flag:
                break
            ip_address = str(ip_int)
            try:
                # Create a socket and set a timeout
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(0.1) # Short timeout for quick scanning
                result = sock.connect_ex((ip_address, target_port))
                if result == 0: # Port is open
                    self.camera_found_signal.emit(ip_address)
                sock.close()
            except Exception as e:
                # print(f"Error scanning {ip_address}:{target_port} - {e}") # Too verbose
                pass
        self.scan_finished_signal.emit()

    def stop(self):
        self._run_flag = False
        self.wait()


from PyQt5.QtCore import QPoint, QRect, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QCursor, QPainter, QPen
from PyQt5.QtWidgets import QLabel


class ROIViewLabel(QLabel):
    """A custom QLabel for displaying an image and drawing/manipulating ROIs."""
    roi_updated = pyqtSignal(int, QRect)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.rois = []  # Stored in IMAGE coordinates
        self.roi_states = []
        self.current_roi_index = -1
        self.interaction_mode = 'none'
        self.handle_size = 8
        self.start_pos = QPoint()
        self.start_rect = QRect()  # Stored in LABEL coordinates during interaction
        self.setMouseTracking(True)

        # Geometry for coordinate mapping
        self.image_rect = QRect()  # The rect of the pixmap within the label
        self.original_image_size = QSize() # Store original image size
        self.scale_x = 1.0
        self.scale_y = 1.0

    def setPixmap(self, pixmap):
        super().setPixmap(pixmap)
        self.original_image_size = pixmap.size() # Store original size
        self._update_geometry()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_geometry()

    def _update_geometry(self):
        """Calculates the geometry of the displayed pixmap for coordinate mapping."""
        if not self.pixmap() or self.pixmap().isNull():
            self.image_rect = QRect()
            return

        pixmap_size = self.pixmap().size()
        label_size = self.size()

        # This mimics QPixmap.scaled(..., Qt.KeepAspectRatio)
        scaled_size = pixmap_size
        scaled_size.scale(label_size, Qt.KeepAspectRatio)

        # This mimics self.setAlignment(Qt.AlignCenter)
        x = (label_size.width() - scaled_size.width()) / 2
        y = (label_size.height() - scaled_size.height()) / 2
        self.image_rect = QRect(QPoint(int(x), int(y)), scaled_size)

        # Calculate scaling factors
        if pixmap_size.width() > 0:
            self.scale_x = scaled_size.width() / pixmap_size.width()
        else:
            self.scale_x = 1.0
            
        if pixmap_size.height() > 0:
            self.scale_y = scaled_size.height() / pixmap_size.height()
        else:
            self.scale_y = 1.0

    def _clamp_rect_to_image_bounds(self, rect):
        """Clamps an image-coordinate QRect to the original image bounds."""
        if not self.original_image_size.isValid():
            return rect

        clamped_left = max(0, rect.left())
        clamped_top = max(0, rect.top())
        clamped_right = min(self.original_image_size.width(), rect.right())
        clamped_bottom = min(self.original_image_size.height(), rect.bottom())

        return QRect(clamped_left, clamped_top, clamped_right - clamped_left, clamped_bottom - clamped_top)

    def map_rect_to_label(self, image_rect):
        """Maps a QRect from image coordinates to label coordinates."""
        if not self.image_rect.isValid():
            return QRect()
        
        label_x = self.image_rect.x() + (image_rect.x() * self.scale_x)
        label_y = self.image_rect.y() + (image_rect.y() * self.scale_y)
        label_w = image_rect.width() * self.scale_x
        label_h = image_rect.height() * self.scale_y
        return QRect(int(label_x), int(label_y), int(label_w), int(label_h))

    def map_rect_from_label(self, label_rect):
        """Maps a QRect from label coordinates to image coordinates."""
        if not self.image_rect.isValid() or self.scale_x == 0 or self.scale_y == 0:
            return QRect()

        image_x = (label_rect.x() - self.image_rect.x()) / self.scale_x
        image_y = (label_rect.y() - self.image_rect.y()) / self.scale_y
        image_w = label_rect.width() / self.scale_x
        image_h = label_rect.height() / self.scale_y
        return QRect(int(image_x), int(image_y), int(image_w), int(image_h))

    def set_rois(self, rois, states):
        """Sets the ROIs (in image coordinates) and their states."""
        self.rois = [QRect(r) for r in rois]
        self.roi_states = states
        self.update()

    def _get_interaction_mode(self, pos, rect_in_label_coords):
        """Determines interaction mode based on cursor position over a label-coordinate rect."""
        margin = self.handle_size
        
        top_left = QRect(rect_in_label_coords.topLeft() - QPoint(margin, margin), rect_in_label_coords.topLeft() + QPoint(margin, margin))
        top_right = QRect(rect_in_label_coords.topRight() - QPoint(-margin, margin), rect_in_label_coords.topRight() + QPoint(-margin, margin))
        bottom_left = QRect(rect_in_label_coords.bottomLeft() - QPoint(margin, -margin), rect_in_label_coords.bottomLeft() + QPoint(margin, -margin))
        bottom_right = QRect(rect_in_label_coords.bottomRight() - QPoint(-margin, -margin), rect_in_label_coords.bottomRight() + QPoint(-margin, -margin))

        if top_left.contains(pos): return 'resize_tl'
        if top_right.contains(pos): return 'resize_tr'
        if bottom_left.contains(pos): return 'resize_bl'
        if bottom_right.contains(pos): return 'resize_br'

        if abs(pos.y() - rect_in_label_coords.top()) < margin and rect_in_label_coords.left() < pos.x() < rect_in_label_coords.right(): return 'resize_t'
        if abs(pos.y() - rect_in_label_coords.bottom()) < margin and rect_in_label_coords.left() < pos.x() < rect_in_label_coords.right(): return 'resize_b'
        if abs(pos.x() - rect_in_label_coords.left()) < margin and rect_in_label_coords.top() < pos.y() < rect_in_label_coords.bottom(): return 'resize_l'
        if abs(pos.x() - rect_in_label_coords.right()) < margin and rect_in_label_coords.top() < pos.y() < rect_in_label_coords.bottom(): return 'resize_r'
        
        if rect_in_label_coords.contains(pos): return 'move'
        return 'none'

    def paintEvent(self, event):
        """Paints the background image and then the ROIs on top."""
        super().paintEvent(event)
        pixmap = self.pixmap()
        if not pixmap or pixmap.isNull():
            return

        painter = QPainter(self)

        for i, image_rect in enumerate(self.rois):
            if self.roi_states[i]:
                label_rect = self.map_rect_to_label(image_rect)
                if not label_rect.isValid():
                    continue

                # Set pen for the rectangle
                pen = QPen(QColor(255, 0, 0, 200), 2, Qt.SolidLine)
                painter.setPen(pen)
                painter.setBrush(Qt.NoBrush)
                painter.drawRect(label_rect)

                # Set font and color for the text
                font = painter.font()
                font.setPointSize(10)
                font.setBold(True)
                painter.setFont(font)
                painter.setPen(QColor(255, 255, 0))

                # Draw the ROI number in the center
                painter.drawText(label_rect, Qt.AlignCenter, f"ROI {i}")

                # Draw handles if this is the currently selected ROI
                if self.interaction_mode != 'none' and i == self.current_roi_index:
                    handle_pen = QPen(QColor(255, 255, 0), 1)
                    painter.setBrush(QColor(255, 255, 0, 150))
                    painter.setPen(handle_pen)
                    s = self.handle_size // 2
                    painter.drawRect(label_rect.topLeft().x() - s, label_rect.topLeft().y() - s, self.handle_size, self.handle_size)
                    painter.drawRect(label_rect.topRight().x() - s, label_rect.topRight().y() - s, self.handle_size, self.handle_size)
                    painter.drawRect(label_rect.bottomLeft().x() - s, label_rect.bottomLeft().y() - s, self.handle_size, self.handle_size)
                    painter.drawRect(label_rect.bottomRight().x() - s, label_rect.bottomRight().y() - s, self.handle_size, self.handle_size)

    def mousePressEvent(self, event):
        """Handles the start of an interaction (move or resize)."""
        if event.button() != Qt.LeftButton:
            return

        for i in reversed(range(len(self.rois))):
            if self.roi_states[i]:
                label_rect = self.map_rect_to_label(self.rois[i])
                mode = self._get_interaction_mode(event.pos(), label_rect)
                if mode != 'none':
                    self.current_roi_index = i
                    self.interaction_mode = mode
                    self.start_pos = event.pos()
                    self.start_rect = QRect(label_rect) # Store the label rect at the start
                    self.update()
                    return
        
        self.current_roi_index = -1
        self.interaction_mode = 'none'

    def mouseMoveEvent(self, event):
        """Handles dragging for move/resize and updates the cursor icon."""
        if self.interaction_mode != 'none' and self.current_roi_index != -1:
            delta = event.pos() - self.start_pos
            new_label_rect = QRect(self.start_rect)

            if self.interaction_mode == 'move':
                new_label_rect.translate(delta)
            elif self.interaction_mode == 'resize_r':
                new_label_rect.setRight(self.start_rect.right() + delta.x())
            elif self.interaction_mode == 'resize_l':
                new_label_rect.setLeft(self.start_rect.left() + delta.x())
            elif self.interaction_mode == 'resize_b':
                new_label_rect.setBottom(self.start_rect.bottom() + delta.y())
            elif self.interaction_mode == 'resize_t':
                new_label_rect.setTop(self.start_rect.top() + delta.y())
            elif self.interaction_mode == 'resize_br':
                new_label_rect.setBottomRight(self.start_rect.bottomRight() + delta)
            elif self.interaction_mode == 'resize_tl':
                new_label_rect.setTopLeft(self.start_rect.topLeft() + delta)
            elif self.interaction_mode == 'resize_tr':
                new_label_rect.setTopRight(self.start_rect.topRight() + delta)
            elif self.interaction_mode == 'resize_bl':
                new_label_rect.setBottomLeft(self.start_rect.bottomLeft() + delta)

            # Convert to image coordinates, clamp, and store
            clamped_image_rect = self._clamp_rect_to_image_bounds(self.map_rect_from_label(new_label_rect))
            self.rois[self.current_roi_index] = clamped_image_rect
            self.update()
        else:
            cursor = Qt.ArrowCursor
            for i in range(len(self.rois)):
                if self.roi_states[i]:
                    label_rect = self.map_rect_to_label(self.rois[i])
                    mode = self._get_interaction_mode(event.pos(), label_rect)
                    if mode != 'none':
                        if mode == 'move': cursor = Qt.SizeAllCursor
                        elif mode in ('resize_t', 'resize_b'): cursor = Qt.SizeVerCursor
                        elif mode in ('resize_l', 'resize_r'): cursor = Qt.SizeHorCursor
                        elif mode in ('resize_tl', 'resize_br'): cursor = Qt.SizeFDiagCursor
                        elif mode in ('resize_tr', 'resize_bl'): cursor = Qt.SizeBDiagCursor
                        break
            self.setCursor(QCursor(cursor))

    def mouseReleaseEvent(self, event):
        """Finalizes the interaction and emits the roi_updated signal."""
        if event.button() == Qt.LeftButton and self.current_roi_index != -1:
            # The ROI in self.rois is already updated and clamped in mouseMoveEvent
            final_image_rect = self.rois[self.current_roi_index].normalized()
            self.rois[self.current_roi_index] = final_image_rect
            
            self.roi_updated.emit(self.current_roi_index, final_image_rect)
            
            self.interaction_mode = 'none'
            self.current_roi_index = -1
            self.update()
''


class RoiSettingsDialog(QDialog):
    def __init__(self, parent=None, camera_config=None, camera_index=None, captured_image=None):
        super().__init__(parent)
        print("RoiSettingsDialog: Initializing...")
        ui_file_path = os.path.join(os.path.dirname(__file__), 'roi_settings_dialog.ui')
        loadUi(ui_file_path, self)
        print("RoiSettingsDialog: UI loaded.")

        self.camera_config = camera_config
        self.camera_index = camera_index

        # Replace the placeholder QLabel with our custom ROIViewLabel
        placeholder = self.findChild(QLabel, "videoFeedLabel")
        self.videoFeedLabel = ROIViewLabel(self)
        self.videoFeedLabel.setObjectName("videoFeedLabel")
        self.videoFeedLabel.setAlignment(Qt.AlignCenter) # Center the image
        
        if captured_image:
            self.videoFeedLabel.setPixmap(captured_image)

        self.verticalLayout.replaceWidget(placeholder, self.videoFeedLabel)
        placeholder.deleteLater()

        self.rois = [QRect(0, 0, 0, 0) for _ in range(10)]
        self.roi_states = [False for _ in range(10)]

        self.buttonBox.accepted.connect(self.apply_roi_settings_to_api)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        # Connect signals for synchronization
        self.roiTableWidget.itemChanged.connect(self.handle_table_change)
        self.videoFeedLabel.roi_updated.connect(self.handle_viewer_change)

        print("RoiSettingsDialog: Calling init_roi_table...")
        self.init_roi_table()
        print("RoiSettingsDialog: init_roi_table finished.")

        print("RoiSettingsDialog: Calling load_roi_settings_from_api...")
        self.load_roi_settings_from_api()
        print("RoiSettingsDialog: load_roi_settings_from_api finished.")

    

    def init_roi_table(self):
        headers = ["Use ROI", "Start X", "Start Y", "End X", "End Y", "Mode", "Condition", "Temperature", "Start Delay", "Stop Delay", "ISO SUE", "ISO Color", "Alarm Out"]
        self.roiTableWidget.setHorizontalHeaderLabels(headers)
        self.roiTableWidget.setRowCount(10) # ROI 0 to ROI 9

        for i in range(10):
            # Add checkbox for "Use ROI"
            checkbox_item = QTableWidgetItem()
            checkbox_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            checkbox_item.setCheckState(Qt.Unchecked)
            self.roiTableWidget.setItem(i, 0, checkbox_item)

            # Add QLineEdit for other columns
            for j in range(1, len(headers)):
                line_edit = QLineEdit()
                self.roiTableWidget.setCellWidget(i, j, line_edit)

    def load_roi_settings_from_api(self):
        ip_address = self.camera_config.get('ip', '')
        user_id = self.camera_config.get('id', '')
        password = self.camera_config.get('password', '')

        if not ip_address or not user_id or not password:
            print("Error: Camera IP, ID, or Password not provided for ROI API call.")
            return

        for i in range(10): # Loop for ROI 0 to ROI 9
            api_url = f"http://{ip_address}/cgi-bin/control/camthermalroi.cgi?id={user_id}&passwd={password}&action=getthermalroi{i}"
            print(f"Attempting to get ROI {i} settings from: {api_url}")

            try:
                response = requests.get(api_url, timeout=5)
                response.raise_for_status() # Raise HTTPError for bad responses (4xx or 5xx)
                response_text = response.text
                print(f"Received ROI {i} data: {response_text}") # Debug print

                roi_data = {}
                for line in response_text.splitlines():
                    if '=' in line:
                        key, value = line.split('=', 1)
                        roi_data[key.strip()] = value.strip()

                # Populate checkbox and update internal state
                checkbox_item = self.roiTableWidget.item(i, 0)
                is_checked = roi_data.get('roi_use') == 'on'
                self.roi_states[i] = is_checked
                checkbox_item.setCheckState(Qt.Checked if is_checked else Qt.Unchecked)

                # Get coordinates and update internal state
                try:
                    start_x = int(roi_data.get('startx', 0))
                    start_y = int(roi_data.get('starty', 0))
                    end_x = int(roi_data.get('endx', 0))
                    end_y = int(roi_data.get('endy', 0))
                    self.rois[i] = QRect(start_x, start_y, end_x - start_x, end_y - start_y)
                except (ValueError, TypeError):
                    # Handle cases where conversion to int fails or data is missing
                    self.rois[i] = QRect(0, 0, 0, 0)

                # Populate QLineEdits
                self.roiTableWidget.cellWidget(i, 1).setText(str(self.rois[i].left()))
                self.roiTableWidget.cellWidget(i, 2).setText(str(self.rois[i].top()))
                self.roiTableWidget.cellWidget(i, 3).setText(str(self.rois[i].right()))
                self.roiTableWidget.cellWidget(i, 4).setText(str(self.rois[i].bottom()))
                if self.roiTableWidget.cellWidget(i, 5): self.roiTableWidget.cellWidget(i, 5).setText(str(roi_data.get('mode', '')))
                if self.roiTableWidget.cellWidget(i, 6): self.roiTableWidget.cellWidget(i, 6).setText(str(roi_data.get('condition', '')))
                if self.roiTableWidget.cellWidget(i, 7): self.roiTableWidget.cellWidget(i, 7).setText(str(roi_data.get('temperature', '')))
                if self.roiTableWidget.cellWidget(i, 8): self.roiTableWidget.cellWidget(i, 8).setText(str(roi_data.get('start_delay', '')))
                if self.roiTableWidget.cellWidget(i, 9): self.roiTableWidget.cellWidget(i, 9).setText(str(roi_data.get('stop_delay', '')))
                if self.roiTableWidget.cellWidget(i, 10): self.roiTableWidget.cellWidget(i, 10).setText(str(roi_data.get('iso_use', '')))
                if self.roiTableWidget.cellWidget(i, 11): self.roiTableWidget.cellWidget(i, 11).setText(str(roi_data.get('iso_color', '')))
                if self.roiTableWidget.cellWidget(i, 12): self.roiTableWidget.cellWidget(i, 12).setText(str(roi_data.get('alarm_out', '')))

            except requests.exceptions.RequestException as e:
                print(f"Error fetching ROI {i} settings: {e}")
                with open("error.txt", "a") as f:
                    f.write(f"\n--- Error fetching ROI {i} settings ---\n")
                    f.write(f"URL: {api_url}\n")
                    f.write(f"Error: {e}\n")
                    f.write("------------------------------------\n")
            except Exception as e:
                print(f"An unexpected error occurred for ROI {i}: {e}")
                import traceback
                with open("error.txt", "a") as f:
                    f.write(f"\n--- Crash during ROI {i} data processing ---\n")
                    f.write(f"URL: {api_url}\n")
                    f.write(f"Error: {e}\n")
                    f.write(traceback.format_exc())
                    f.write("------------------------------------\n")

        self.videoFeedLabel.set_rois(self.rois, self.roi_states)
        print("All ROI settings loaded successfully (or attempted to load).")

    def handle_table_change(self, item):
        """Handles changes in the ROI table and updates the ROI viewer."""
        row = item.row()
        # Temporarily disconnect signal to prevent recursion
        self.roiTableWidget.itemChanged.disconnect(self.handle_table_change)

        try:
            if item.column() == 0: # Checkbox for 'Use ROI'
                self.roi_states[row] = (item.checkState() == Qt.Checked)
            else:
                # Update rectangle from table values
                try:
                    start_x = int(self.roiTableWidget.cellWidget(row, 1).text())
                    start_y = int(self.roiTableWidget.cellWidget(row, 2).text())
                    end_x = int(self.roiTableWidget.cellWidget(row, 3).text())
                    end_y = int(self.roiTableWidget.cellWidget(row, 4).text())
                    self.rois[row] = QRect(start_x, start_y, end_x - start_x, end_y - start_y)
                except (ValueError, TypeError):
                    pass # Ignore errors from invalid input

            self.videoFeedLabel.set_rois(self.rois, self.roi_states)

        finally:
            # Reconnect signal
            self.roiTableWidget.itemChanged.connect(self.handle_table_change)

    def handle_viewer_change(self, index, rect):
        """Handles ROI updates from the viewer and updates the table."""
        # Temporarily disconnect signal to prevent recursion
        self.roiTableWidget.itemChanged.disconnect(self.handle_table_change)

        try:
            self.rois[index] = rect
            self.roiTableWidget.cellWidget(index, 1).setText(str(rect.left()))
            self.roiTableWidget.cellWidget(index, 2).setText(str(rect.top()))
            self.roiTableWidget.cellWidget(index, 3).setText(str(rect.right()))
            self.roiTableWidget.cellWidget(index, 4).setText(str(rect.bottom()))

        finally:
            # Reconnect signal
            self.roiTableWidget.itemChanged.connect(self.handle_table_change)

    def apply_roi_settings_to_api(self):
        ip_address = self.camera_config.get('ip', '')
        user_id = self.camera_config.get('id', '')
        password = self.camera_config.get('password', '')

        if not ip_address or not user_id or not password:
            print("Error: Camera IP, ID, or Password not provided for ROI API call.")
            return

        all_roi_settings = self.get_roi_settings()

        for i, roi_data in enumerate(all_roi_settings):
            # Construct API URL parameters
            params = {
                'id': user_id,
                'passwd': password,
                'action': f'setthermalroi{i}',
                'roi_use': roi_data['roi_use'],
                'startx': roi_data['start_x'],
                'starty': roi_data['start_y'],
                'endx': roi_data['end_x'],
                'endy': roi_data['end_y'],
                'mode': roi_data['mode'],
                'condition': roi_data['condition'],
                'temperature': roi_data['temperature'],
                'start_delay': roi_data['start_delay'],
                'stop_delay': roi_data['stop_delay'],
                'iso_use': roi_data['iso_sue'], # Note: API uses iso_use, table uses iso_sue
                'iso_color': roi_data['iso_color'],
                'alarm_out': roi_data['alarm_out'],
            }

            api_url = f"http://{ip_address}/cgi-bin/control/camthermalroi.cgi"
            print(f"Attempting to set ROI {i} settings: {api_url}?{requests.compat.urlencode(params)}")

            try:
                response = requests.get(api_url, params=params, timeout=5)
                response.raise_for_status() # Raise HTTPError for bad responses (4xx or 5xx)
                print(f"Successfully set ROI {i} data: {response.text}")
            except requests.exceptions.RequestException as e:
                print(f"Error setting ROI {i} settings: {e}")
                with open("error.txt", "a") as f:
                    f.write(f"\n--- Error setting ROI {i} settings ---\n")
                    f.write(f"URL: {api_url}?{requests.compat.urlencode(params)}\n")
                    f.write(f"Error: {e}\n")
                    f.write(traceback.format_exc())
                    f.write("------------------------------------\n")
            except Exception as e:
                print(f"An unexpected error occurred while setting ROI {i}: {e}")
                import traceback
                with open("error.txt", "a") as f:
                    f.write(f"\n--- Crash while setting ROI {i} data ---\n")
                    f.write(f"URL: {api_url}?{requests.compat.urlencode(params)}\n")
                    f.write(f"Error: {e}\n")
                    f.write(traceback.format_exc())
                    f.write("------------------------------------\n")

        print("All ROI settings applied successfully (or attempted to apply).")

    def get_roi_settings(self):
        roi_settings = []
        for i in range(10):
            row_data = {}
            # Get checkbox state
            checkbox_item = self.roiTableWidget.item(i, 0)
            row_data['roi_use'] = "on" if checkbox_item.checkState() == Qt.Checked else "off"

            # Get values from QLineEdit widgets
            row_data['start_x'] = self.roiTableWidget.cellWidget(i, 1).text()
            row_data['start_y'] = self.roiTableWidget.cellWidget(i, 2).text()
            row_data['end_x'] = self.roiTableWidget.cellWidget(i, 3).text()
            row_data['end_y'] = self.roiTableWidget.cellWidget(i, 4).text()
            row_data['mode'] = self.roiTableWidget.cellWidget(i, 5).text()
            row_data['condition'] = self.roiTableWidget.cellWidget(i, 6).text()
            row_data['temperature'] = self.roiTableWidget.cellWidget(i, 7).text()
            row_data['start_delay'] = self.roiTableWidget.cellWidget(i, 8).text()
            row_data['stop_delay'] = self.roiTableWidget.cellWidget(i, 9).text()
            row_data['iso_sue'] = self.roiTableWidget.cellWidget(i, 10).text()
            row_data['iso_color'] = self.roiTableWidget.cellWidget(i, 11).text()
            row_data['alarm_out'] = self.roiTableWidget.cellWidget(i, 12).text()
            roi_settings.append(row_data)
        return roi_settings


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # Load the UI file
        ui_file_path = os.path.join(os.path.dirname(__file__), 'thermal_camera_viewer.ui')
        loadUi(ui_file_path, self)

        self.actionManage_Cameras.triggered.connect(self.show_camera_manager)

        self.camera_threads = {} # Dictionary to store video threads for each camera
        self.camera_labels = {
            0: self.videoFeedLabel_1,
            1: self.videoFeedLabel_2,
            2: self.videoFeedLabel_3,
            3: self.videoFeedLabel_4,
        }
        self.camera_configs = [] # List to store camera configurations from CameraManagerDialog
        self.active_camera_controls = {} # Dictionary to store dynamically created camera control widgets

        # Initial setup: rebuild controls based on empty config or saved config
        self.rebuild_camera_controls()

    def show_camera_manager(self):
        dialog = CameraManagerDialog(self, initial_configs=self.camera_configs)
        if dialog.exec_() == QDialog.Accepted:
            self.camera_configs = dialog.get_camera_configs()
            self.rebuild_camera_controls()
            # Automatically connect to cameras after configs are updated
            for i, config in enumerate(self.camera_configs):
                if i < 4: # Ensure we only connect to the first 4 cameras
                    self.connect_to_camera(config['ip'], config['id'], config['password'], i)

    def rebuild_camera_controls(self):
        # Clear existing camera controls
        for widget in self.active_camera_controls.values():
            widget.deleteLater()
        self.active_camera_controls.clear()

        # Clear existing video feeds
        for label in self.camera_labels.values():
            label.clear()
            label.setText("Video Feed") # Reset text

        # Stop all running video threads
        for camera_id, thread in self.camera_threads.items():
            if thread.isRunning():
                thread.stop()
                thread.wait()
        self.camera_threads.clear()

        # Rebuild controls based on updated camera_configs
        for i, config in enumerate(self.camera_configs):
            if i >= 4: # Limit to 4 cameras for now
                print(f"Warning: Only supporting up to 4 cameras. Skipping Camera {i+1}.")
                break

            camera_group_box = QGroupBox(f"Camera {i+1}")
            form_layout = QFormLayout()

            ip_label = QLabel("IP Address:")
            ip_lineEdit = QLineEdit(config.get('ip', ''))
            ip_lineEdit.setAlignment(Qt.AlignCenter)

            nuc_button = QPushButton("Run NUC")
            roi_button = QPushButton("Set ROI")

            form_layout.addRow(ip_label, ip_lineEdit)
            form_layout.addRow(nuc_button)
            form_layout.addRow(roi_button)

            camera_group_box.setLayout(form_layout)
            self.cameraSettingsLayout.addWidget(camera_group_box)
            self.active_camera_controls[i] = camera_group_box # Store reference

            # Connect signals using lambda to pass specific line edits and camera_index
            nuc_button.clicked.connect(lambda checked, ip=config.get('ip', ''), _id=config.get('id', ''), pw=config.get('password', ''): self.run_nuc(ip, _id, pw, checked))
            roi_button.clicked.connect(lambda checked, config=config, index=i: self.show_roi_settings_dialog(config, index))

    def connect_to_camera(self, ip_address, user_id, password, camera_index):
        # Use a combination of ip_address and camera_index as a unique identifier
        camera_id = f"{ip_address}_{camera_index}"

        if camera_id in self.camera_threads and self.camera_threads[camera_id].isRunning():
            self.camera_threads[camera_id].stop()
            self.camera_threads[camera_id].wait()

        port = 554 # Fixed port

        rtsp_url = f"rtsp://{user_id}:{password}@{ip_address}:{port}/stream1"
        print(f"Attempting to connect to: {rtsp_url}")

        video_thread = VideoThread(rtsp_url, camera_index)
        video_thread.change_pixmap_signal.connect(self.update_image)
        video_thread.start()
        self.camera_threads[camera_id] = video_thread

    def run_nuc(self, ip_address, user_id, password, checked):

        nuc_url = f"http://{ip_address}/cgi-bin/control/camthermalfunc.cgi?id={user_id}&passwd={password}&action=setthermalfunc&run_nuc=on"
        print(f"Attempting to run NUC: {nuc_url}")

        try:
            response = requests.get(nuc_url, auth=(user_id, password), timeout=5)
            response.raise_for_status()
            print("\n--- NUC Command Response ---")
            if response.status_code == 200:
                print("NUC command sent successfully.")
            else:
                print(f"NUC command returned status code: {response.status_code}")
                print(f"Response: {response.text}")
            print("----------------------------")
        except requests.exceptions.RequestException as e:
            print(f"Error running NUC: {e}")

    

    def update_image(self, qt_image, camera_index):
        if camera_index in self.camera_labels:
            self.camera_labels[camera_index].setPixmap(QPixmap.fromImage(qt_image).scaled(self.camera_labels[camera_index].size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def closeEvent(self, event):
        for camera_id, thread in self.camera_threads.items():
            if thread.isRunning():
                thread.stop()
                thread.wait()
        event.accept()

    def show_roi_settings_dialog(self, camera_config, camera_index):
        if camera_index in self.camera_labels and self.camera_labels[camera_index].pixmap():
            # Scale the captured image to a fixed size (e.g., 640x480) before passing to ROI dialog
            captured_image = self.camera_labels[camera_index].pixmap().scaled(640, 480, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        else:
            captured_image = None

        dialog = RoiSettingsDialog(self, camera_config=camera_config, camera_index=camera_index, captured_image=captured_image)
        if dialog.exec_() == QDialog.Accepted:
            roi_settings = dialog.get_roi_settings()
            # TODO: Save ROI settings back to camera_config or process them
            print(f"ROI Settings for Camera {camera_index}: {roi_settings}")


if __name__ == '__main__':
    def custom_exception_hook(exctype, value, traceback_obj):
        import traceback
        error_message = f"\n--- Uncaught Exception ---\n"
        error_message += f"Type: {exctype.__name__}\n"
        error_message += f"Value: {value}\n"
        error_message += f"Traceback:\n{traceback.format_tb(traceback_obj)}"
        error_message += "--------------------------\n"
        with open("error.txt", "a") as f:
            f.write(error_message)
        sys.__excepthook__(exctype, value, traceback_obj) # Call default hook

    sys.excepthook = custom_exception_hook

    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())