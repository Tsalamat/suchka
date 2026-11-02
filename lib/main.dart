import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import 'lane_ffi.dart';

const _kSolid = Color(0xFFFFB300); // сплошная
const _kDashed = Color(0xFF00E676); // прерывистая

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await SystemChrome.setPreferredOrientations(
      [DeviceOrientation.landscapeLeft]);
  await SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersiveSticky);
  runApp(MaterialApp(
    debugShowCheckedModeBanner: false,
    theme: ThemeData.dark(useMaterial3: true),
    home: const LaneScreen(),
  ));
}

class LaneScreen extends StatefulWidget {
  const LaneScreen({super.key});

  @override
  State<LaneScreen> createState() => _LaneScreenState();
}

class _LaneScreenState extends State<LaneScreen> {
  CameraController? _cam;
  String? _error;

  bool _busy = false;
  DateTime _last = DateTime.fromMillisecondsSinceEpoch(0);
  static const _minFrameGap = Duration(milliseconds: 100);

  List<int> _segments = const [];
  int _frameW = 16, _frameH = 9;
  int _procMs = 0;
  double _fps = 0;
  DateTime _lastResult = DateTime.now();

  double _sens = 60; // 0..100
  bool _showRoi = false;
  bool _panel = true;
  String _clock = '';
  Timer? _timer;

  int get _threshold => (130 - _sens * 1.1).round();

  @override
  void initState() {
    super.initState();
    _tick();
    _timer = Timer.periodic(const Duration(seconds: 1), (_) => _tick());
    _init();
  }

  void _tick() {
    final n = DateTime.now();
    String two(int v) => v.toString().padLeft(2, '0');
    if (mounted) {
      setState(() => _clock =
          '${two(n.day)}.${two(n.month)}.${n.year}  ${two(n.hour)}:${two(n.minute)}:${two(n.second)}');
    }
  }

  Future<void> _init() async {
    try {
      final cams = await availableCameras();
      if (cams.isEmpty) {
        setState(() => _error = 'Камера не найдена');
        return;
      }
      final cam = cams.firstWhere(
          (c) => c.lensDirection == CameraLensDirection.back,
          orElse: () => cams.first);
      final c = CameraController(
        cam,
        ResolutionPreset.high, // 1280x720
        enableAudio: false,
        imageFormatGroup: ImageFormatGroup.yuv420,
      );
      await c.initialize();
      await c.startImageStream(_onFrame);
      if (!mounted) return;
      setState(() => _cam = c);
    } on CameraException catch (e) {
      setState(() => _error = 'Ошибка камеры: ${e.description ?? e.code}');
    } catch (e) {
      setState(() => _error = 'Ошибка: $e');
    }
  }

  void _onFrame(CameraImage img) {
    final now = DateTime.now();
    if (_busy || now.difference(_last) < _minFrameGap) return;
    _busy = true;
    _last = now;

    final plane = img.planes[0];
    final bytes = Uint8List.fromList(plane.bytes);
    final w = img.width, h = img.height, stride = plane.bytesPerRow;

    detectLanes(
      yPlane: bytes,
      width: w,
      height: h,
      rowStride: stride,
      threshold: _threshold,
    ).then((r) {
      if (!mounted) return;
      final t = DateTime.now();
      final dt = t.difference(_lastResult).inMicroseconds / 1e6;
      _lastResult = t;
      setState(() {
        _segments = r.segments;
        _frameW = w;
        _frameH = h;
        _procMs = r.millis;
        if (dt > 0) _fps = _fps == 0 ? 1 / dt : _fps * 0.8 + (1 / dt) * 0.2;
      });
    }).catchError((Object e) {
      if (mounted) setState(() => _error = 'Ошибка обработки: $e');
    }).whenComplete(() => _busy = false);
  }

  @override
  void dispose() {
    _timer?.cancel();
    _cam?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final cam = _cam;
    if (_error != null) {
      return Scaffold(
          body: Center(
              child: Padding(
        padding: const EdgeInsets.all(24),
        child: Text(_error!, style: const TextStyle(fontSize: 18)),
      )));
    }
    if (cam == null || !cam.value.isInitialized) {
      return const Scaffold(body: Center(child: CircularProgressIndicator()));
    }

    final ps = cam.value.previewSize!;
    final pw = math.max(ps.width, ps.height);
    final ph = math.min(ps.width, ps.height);

    int solid = 0, dashed = 0;
    for (var i = 0; i + 4 < _segments.length; i += kSegStride) {
      if (_segments[i + 4] == 1) {
        solid++;
      } else {
        dashed++;
      }
    }
    final found = solid + dashed > 0;

    return Scaffold(
      backgroundColor: Colors.black,
      body: Stack(fit: StackFit.expand, children: [
        // Камера на весь экран (cover), разметка в тех же координатах.
        FittedBox(
          fit: BoxFit.cover,
          clipBehavior: Clip.hardEdge,
          child: SizedBox(
            width: pw,
            height: ph,
            child: Stack(fit: StackFit.expand, children: [
              CameraPreview(cam),
              CustomPaint(
                  painter: _LanePainter(_segments, _frameW, _frameH, _showRoi)),
            ]),
          ),
        ),
        const IgnorePointer(child: CustomPaint(painter: _CornersPainter())),
        // HUD
        Positioned(
          left: 0,
          right: 0,
          top: 0,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 6),
            decoration: const BoxDecoration(
              gradient: LinearGradient(
                  begin: Alignment.topCenter,
                  end: Alignment.bottomCenter,
                  colors: [Color(0xCC000000), Color(0x00000000)]),
            ),
            child: Row(children: [
              Icon(Icons.circle,
                  size: 12, color: found ? _kDashed : Colors.redAccent),
              const SizedBox(width: 8),
              Text(found ? 'РАЗМЕТКА ОБНАРУЖЕНА' : 'РАЗМЕТКА НЕ ОБНАРУЖЕНА',
                  style: const TextStyle(
                      fontWeight: FontWeight.w700, letterSpacing: 0.5)),
              const SizedBox(width: 16),
              _Legend(_kSolid, 'сплошная $solid'),
              const SizedBox(width: 10),
              _Legend(_kDashed, 'прерывистая $dashed'),
              const Spacer(),
              Text('${_fps.toStringAsFixed(1)} к/с • $_procMs мс',
                  style: const TextStyle(fontSize: 13)),
              const SizedBox(width: 14),
              Text(_clock,
                  style: const TextStyle(
                      fontSize: 13, fontFeatures: [FontFeature.tabularFigures()])),
            ]),
          ),
        ),
        // Панель настроек
        Positioned(
          left: 12,
          right: 12,
          bottom: 8,
          child: _panel
              ? Container(
                  padding: const EdgeInsets.symmetric(horizontal: 12),
                  decoration: BoxDecoration(
                      color: Colors.black54,
                      borderRadius: BorderRadius.circular(12)),
                  child: Row(children: [
                    const Text('Чувствительность'),
                    Expanded(
                      child: Slider(
                        min: 0,
                        max: 100,
                        value: _sens,
                        onChanged: (v) => setState(() => _sens = v),
                      ),
                    ),
                    Text('${_sens.round()}'),
                    const SizedBox(width: 10),
                    FilterChip(
                      label: const Text('Зона'),
                      selected: _showRoi,
                      onSelected: (v) => setState(() => _showRoi = v),
                    ),
                    IconButton(
                        icon: const Icon(Icons.keyboard_arrow_down),
                        onPressed: () => setState(() => _panel = false)),
                  ]),
                )
              : Align(
                  alignment: Alignment.bottomRight,
                  child: IconButton.filledTonal(
                      icon: const Icon(Icons.tune),
                      onPressed: () => setState(() => _panel = true)),
                ),
        ),
      ]),
    );
  }
}

class _Legend extends StatelessWidget {
  final Color color;
  final String text;
  const _Legend(this.color, this.text);

  @override
  Widget build(BuildContext context) => Row(children: [
        Container(width: 18, height: 4, color: color),
        const SizedBox(width: 5),
        Text(text, style: const TextStyle(fontSize: 13)),
      ]);
}

class _LanePainter extends CustomPainter {
  final List<int> seg;
  final int fw, fh;
  final bool roi;
  _LanePainter(this.seg, this.fw, this.fh, this.roi);

  @override
  void paint(Canvas canvas, Size size) {
    final sx = size.width / fw, sy = size.height / fh;
    if (roi) {
      final top = kRoiTop * size.height, bottom = kRoiBottom * size.height;
      final path = Path()
        ..moveTo(kRoiTopLeft * size.width, top)
        ..lineTo(kRoiTopRight * size.width, top)
        ..lineTo(size.width, bottom)
        ..lineTo(0, bottom)
        ..close();
      canvas.drawPath(
          path,
          Paint()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 2
            ..color = const Color(0x88FFFFFF));
    }
    final shadow = Paint()
      ..color = const Color(0xAA000000)
      ..strokeWidth = 9
      ..strokeCap = StrokeCap.round;
    final p = Paint()
      ..strokeWidth = 5
      ..strokeCap = StrokeCap.round;
    for (var i = 0; i + 4 < seg.length; i += kSegStride) {
      final a = Offset(seg[i] * sx, seg[i + 1] * sy);
      final b = Offset(seg[i + 2] * sx, seg[i + 3] * sy);
      p.color = seg[i + 4] == 1 ? _kSolid : _kDashed;
      canvas.drawLine(a, b, shadow);
      canvas.drawLine(a, b, p);
    }
  }

  @override
  bool shouldRepaint(_LanePainter old) => true;
}

class _CornersPainter extends CustomPainter {
  const _CornersPainter();

  @override
  void paint(Canvas canvas, Size s) {
    final p = Paint()
      ..color = const Color(0xAAFFFFFF)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 3;
    const m = 14.0, l = 36.0;
    void corner(double x, double y, double dx, double dy) {
      canvas.drawLine(Offset(x, y), Offset(x + dx * l, y), p);
      canvas.drawLine(Offset(x, y), Offset(x, y + dy * l), p);
    }

    corner(m, m + 36, 1, 1);
    corner(s.width - m, m + 36, -1, 1);
    corner(m, s.height - m, 1, -1);
    corner(s.width - m, s.height - m, -1, -1);
  }

  @override
  bool shouldRepaint(_CornersPainter old) => false;
}
