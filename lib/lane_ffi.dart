import 'dart:ffi';
import 'dart:isolate';
import 'dart:typed_data';

import 'package:ffi/ffi.dart';

typedef _DetectC = Int32 Function(Pointer<Uint8>, Int32, Int32, Int32, Int32,
    Int32, Pointer<Int32>, Int32);
typedef _DetectDart = int Function(
    Pointer<Uint8>, int, int, int, int, int, Pointer<Int32>, int);

const int kMaxSegments = 24;
const int kSegStride = 5; // x1, y1, x2, y2, kind (0 = прерывистая, 1 = сплошная)

// Должны совпадать с константами в rust/src/lib.rs
const double kRoiTop = 0.42;
const double kRoiBottom = 0.96;
const double kRoiTopLeft = 0.20;
const double kRoiTopRight = 0.80;

class DetectResult {
  final List<int> segments; // плоский список, по kSegStride чисел на отрезок
  final int millis;
  const DetectResult(this.segments, this.millis);
}

/// Запускает Rust-детектор в отдельном изоляте, чтобы не тормозить интерфейс.
Future<DetectResult> detectLanes({
  required Uint8List yPlane,
  required int width,
  required int height,
  required int rowStride,
  required int threshold,
}) {
  return Isolate.run(() {
    final sw = Stopwatch()..start();
    final lib = DynamicLibrary.open('liblane_native.so');
    final fn = lib.lookupFunction<_DetectC, _DetectDart>('lane_detect');
    final input = calloc<Uint8>(yPlane.length);
    final out = calloc<Int32>(kMaxSegments * kSegStride);
    try {
      input.asTypedList(yPlane.length).setAll(0, yPlane);
      final n = fn(input, yPlane.length, width, height, rowStride, threshold,
          out, kMaxSegments);
      final segs =
          n <= 0 ? <int>[] : out.asTypedList(n * kSegStride).toList();
      return DetectResult(segs, sw.elapsedMilliseconds);
    } finally {
      calloc.free(input);
      calloc.free(out);
    }
  });
}
