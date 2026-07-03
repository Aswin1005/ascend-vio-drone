import cv2
import threading
import queue
import time


class SurveyRunner:
    """Threaded runner for SurveyPipeline.

    Thread model:
      - Capture thread: reads frames from camera/ROS topic at target_fps rate.
        Sleeps between reads so it does NOT busy-spin and steal CPU from OpenVINS.
      - Main thread: pulls frames from the queue, runs SurveyPipeline.process(),
        handles display and saves.

    The capture thread passes two frames to the queue:
      (hd_frame_original, hd_frame_working)
      - hd_frame_original: untouched 1280×720 — saved to disk on detection.
      - hd_frame_working:  resized to config.input_width × config.input_height
                           — used for ORB patch extraction (may be same as original).
    """

    def __init__(self, capture, pipeline, config):
        self.capture = capture
        self.pipeline = pipeline
        self.config = config

        # maxsize=1: always work on the freshest frame; drop anything older.
        self.frame_queue = queue.Queue(maxsize=1)
        self.running = False
        self.capture_thread = None

        self.total_captured = 0
        self.total_dropped = 0

    def _capture_loop(self):
        target_fps = getattr(self.config, 'target_fps', 1.0)
        frame_interval = 1.0 / max(target_fps, 0.1)

        while self.running:
            t_start = time.perf_counter()

            packet = self.capture.read()
            if packet is None:
                try:
                    self.frame_queue.put(None, timeout=1.0)
                except queue.Full:
                    pass
                break

            hd_original = packet.color  # always keep the original (e.g. 1280×720)
            self.total_captured += 1

            # Resize to working resolution if needed
            fh, fw = hd_original.shape[:2]
            need_w = self.config.input_width
            need_h = self.config.input_height
            if need_w > 0 and need_h > 0 and (fw != need_w or fh != need_h):
                hd_working = cv2.resize(
                    hd_original, (need_w, need_h), interpolation=cv2.INTER_AREA
                )
            else:
                hd_working = hd_original

            # Drop-oldest policy
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                    self.total_dropped += 1
                except queue.Empty:
                    pass

            try:
                self.frame_queue.put_nowait((hd_original, hd_working))
            except queue.Full:
                self.total_dropped += 1

            elapsed = time.perf_counter() - t_start
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def run(self):
        """Main blocking loop. Starts capture thread, processes frames."""
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        target_fps = getattr(self.config, 'target_fps', 1.0)
        print("[runner] survey started")
        print(f"[runner] seeds loaded: {len(self.pipeline.seeds)}")
        print(f"[runner] working resolution: {self.config.input_width}x{self.config.input_height}")
        print(f"[runner] LR match size: {self.config.lr_size}x{self.config.lr_size} (rulebook)")
        print(f"[runner] patch scales: {self.config.patch_sizes}px")
        print(f"[runner] target fps: {target_fps:.1f}")
        print(f"[runner] blur threshold: {self.config.blur_threshold}")
        print()

        try:
            while self.running:
                try:
                    item = self.frame_queue.get(timeout=2.0)
                except queue.Empty:
                    continue

                if item is None:
                    print("[runner] end of stream")
                    break

                hd_original, hd_working = item
                result = self.pipeline.process(hd_working, hd_frame_original=hd_original)

                if self.config.display:
                    canvas = self.pipeline.draw_overlay(hd_working, result)
                    stats = (f"captured:{self.total_captured} | "
                             f"dropped:{self.total_dropped} | "
                             f"processed:{self.pipeline.frames_processed}")
                    cv2.putText(canvas, stats, (10, canvas.shape[0] - 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                    cv2.imshow("IRoC Survey", canvas)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord('q')):
                        print("[runner] user quit")
                        break

                if self.config.stop_on_any_match and self.pipeline.confirmed is not None:
                    print("[runner] target confirmed — stopping.")
                    if self.config.display:
                        cv2.waitKey(3000)
                    break

        except KeyboardInterrupt:
            print("[runner] interrupted")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.capture_thread is not None:
            self.capture_thread.join(timeout=3.0)
        self.capture.release()
        cv2.destroyAllWindows()

        print()
        print("=" * 50)
        print("SURVEY SUMMARY")
        print("=" * 50)
        print(f"  Frames captured:  {self.total_captured}")
        print(f"  Frames dropped:   {self.total_dropped}")
        print(f"  Frames processed: {self.pipeline.frames_processed}")
        if self.pipeline.confirmed:
            det = self.pipeline.confirmed
            print(f"  Target found: {det.seed_name}  inliers={det.inliers}  score={det.score:.2f}")
        else:
            print("  Target: NOT found")
        print("=" * 50)
