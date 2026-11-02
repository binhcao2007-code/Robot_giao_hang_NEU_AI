import heapq
import time
from collections import deque
from functools import lru_cache
from itertools import count

# Ma tran 10x10: 0 = duong di, 1 = vat can, 2 = diem xuat phat, 3 = diem giao hang
CAMPUS_MAP = [
    [2, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 1, 1, 0, 1, 0, 1, 1, 1, 0],
    [0, 1, 0, 0, 0, 0, 0, 0, 3, 0],
    [0, 1, 0, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
    [1, 1, 0, 1, 3, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 1, 1, 1, 1, 0],
    [0, 3, 0, 1, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 3],
]

START = (0, 0)
GOALS = {
    "C1 (A2-101)": (2, 8),
    "C2 (B-204)": (5, 4),
    "C3 (C-302)": (8, 1),
    "C4 (D2-405)": (9, 9),
}

# Thu tu nay duoc giu co dinh de ket qua co the lap lai.
MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]
REPEATS = 50
GRID_SCALE = 3


def manhattan(a, b):
    """Khoang cach Manhattan giua hai toa do."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def valid_neighbors(grid, pos):
    """Tra ve cac o ke hop le theo 4 huong."""
    rows, cols = len(grid), len(grid[0])
    r, c = pos
    for dr, dc in MOVES:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] != 1:
            yield (nr, nc)


def reconstruct_path(parent, end_state):
    """Truy vet mang cha de lay duong di."""
    path = []
    state = end_state
    while state is not None:
        path.append(state)
        state = parent[state]
    path.reverse()
    return path


def make_result(path, expanded, open_max):
    return {
        "path": path,
        "steps": len(path) - 1 if path else None,
        "expanded": expanded,
        "open_max": open_max,
    }


def bfs_search(grid, start, goal):
    """BFS: dung hang doi FIFO, toi uu khi moi buoc co chi phi bang 1."""
    queue = deque([start])
    visited = {start}
    parent = {start: None}
    expanded = 0
    open_max = 1

    while queue:
        current = queue.popleft()
        expanded += 1  # Tinh ca node dich duoc lay ra khoi OPEN.

        if current == goal:
            return make_result(reconstruct_path(parent, goal), expanded, open_max)

        for nxt in valid_neighbors(grid, current):
            if nxt not in visited:
                visited.add(nxt)
                parent[nxt] = current
                queue.append(nxt)

        open_max = max(open_max, len(queue))

    return make_result(None, expanded, open_max)


def informed_search(grid, start, goal, algorithm):
    """GBFS uu tien h(n); A* uu tien g(n) + h(n)."""
    if algorithm not in {"GBFS", "A*"}:
        raise ValueError("algorithm phai la 'GBFS' hoac 'A*'")

    serial = count()
    h_start = manhattan(start, goal)
    # Heap: (f, h, thu_tu_chen, g, toa_do)
    start_f = h_start if algorithm == "GBFS" else h_start
    open_heap = [(start_f, h_start, next(serial), 0, start)]
    open_active = {start}
    best_g = {start: 0}
    parent = {start: None}
    closed = set()
    expanded = 0
    open_max = 1

    while open_heap:
        f_value, h_value, _, g_value, current = heapq.heappop(open_heap)

        # Bo qua phan tu cu neu A* da tim duoc duong tot hon den cung node.
        if current not in open_active or g_value != best_g.get(current):
            continue

        open_active.remove(current)
        if current in closed:
            continue

        closed.add(current)
        expanded += 1  # Tinh ca node dich duoc lay ra khoi OPEN.

        if current == goal:
            return make_result(reconstruct_path(parent, goal), expanded, open_max)

        for nxt in valid_neighbors(grid, current):
            new_g = best_g[current] + 1
            h_next = manhattan(nxt, goal)

            if algorithm == "GBFS":
                # GBFS khong dung g(n) de xep thu tu; moi node chi duoc dua vao mot lan.
                if nxt in closed or nxt in open_active:
                    continue
                new_f = h_next
            else:
                # A* cap nhat neu tim thay mot duong co g(n) tot hon.
                if new_g >= best_g.get(nxt, float("inf")):
                    continue
                best_g[nxt] = new_g
                parent[nxt] = current
                if nxt in closed:
                    closed.remove(nxt)
                new_f = new_g + h_next
                open_active.add(nxt)
                heapq.heappush(
                    open_heap,
                    (new_f, h_next, next(serial), new_g, nxt),
                )
                open_max = max(open_max, len(open_active))
                continue

            # Nhanh GBFS
            best_g[nxt] = new_g
            parent[nxt] = current
            open_active.add(nxt)
            heapq.heappush(
                open_heap,
                (new_f, h_next, next(serial), new_g, nxt),
            )
            open_max = max(open_max, len(open_active))

    return make_result(None, expanded, open_max)


def bfs_multi_goal(grid, start, goals):
    """BFS tren state (vi_tri, bitmask_cac_diem_chua_giao)."""
    goal_positions = tuple(goals.values())
    goal_to_bit = {pos: 1 << i for i, pos in enumerate(goal_positions)}
    all_mask = (1 << len(goal_positions)) - 1

    start_mask = all_mask
    if start in goal_to_bit:
        start_mask &= ~goal_to_bit[start]

    start_state = (start, start_mask)
    queue = deque([start_state])
    visited = {start_state}
    parent = {start_state: None}
    expanded = 0
    open_max = 1

    while queue:
        state = queue.popleft()
        pos, remaining_mask = state
        expanded += 1

        if remaining_mask == 0:
            state_path = reconstruct_path(parent, state)
            position_path = [item[0] for item in state_path]
            return make_result(position_path, expanded, open_max)

        for nxt in valid_neighbors(grid, pos):
            new_mask = remaining_mask & ~goal_to_bit.get(nxt, 0)
            next_state = (nxt, new_mask)
            if next_state not in visited:
                visited.add(next_state)
                parent[next_state] = state
                queue.append(next_state)

        open_max = max(open_max, len(queue))

    return make_result(None, expanded, open_max)


def gbfs_nearest_multi_goal(grid, start, goals):
    """GBFS nhieu diem: moi chang chon diem chua giao gan nhat theo Manhattan."""
    remaining = set(goals.values())
    current = start
    full_path = [start]
    total_expanded = 0
    overall_open_max = 1

    while remaining:
        next_goal = min(
            remaining,
            key=lambda goal: (manhattan(current, goal), goal),
        )
        result = informed_search(grid, current, next_goal, "GBFS")
        total_expanded += result["expanded"]
        overall_open_max = max(overall_open_max, result["open_max"])

        if not result["path"]:
            return make_result(None, total_expanded, overall_open_max)

        # Bo sung duong di, khong lap lai vi tri bat dau cua chang moi.
        full_path.extend(result["path"][1:])
        current = result["path"][-1]

        # Neu duong di di qua diem giao khac thi cung tinh la da giao diem do.
        visited_positions = set(result["path"])
        remaining.difference_update(visited_positions)

    return make_result(full_path, total_expanded, overall_open_max)


def make_mst_heuristic(goals):
    """Tao heuristic MST Manhattan cho bai toan giao hang nhieu diem."""
    goal_positions = tuple(goals.values())
    goal_to_bit = {pos: 1 << i for i, pos in enumerate(goal_positions)}

    @lru_cache(maxsize=None)
    def mst_cost(current, remaining_mask):
        nodes = [current]
        for i, goal in enumerate(goal_positions):
            if remaining_mask & (1 << i):
                nodes.append(goal)

        if len(nodes) <= 1:
            return 0

        # Prim: tinh cay khung nho nhat tren khoang cach Manhattan.
        used = {0}
        best_edge = [float("inf")] * len(nodes)
        for i in range(1, len(nodes)):
            best_edge[i] = manhattan(nodes[0], nodes[i])

        total = 0
        while len(used) < len(nodes):
            next_index = min(
                (i for i in range(len(nodes)) if i not in used),
                key=lambda i: best_edge[i],
            )
            total += best_edge[next_index]
            used.add(next_index)
            for i in range(len(nodes)):
                if i not in used:
                    best_edge[i] = min(
                        best_edge[i],
                        manhattan(nodes[next_index], nodes[i]),
                    )
        return total

    def heuristic(current, remaining_mask):
        return mst_cost(current, remaining_mask)

    return goal_to_bit, heuristic


def astar_mst_multi_goal(grid, start, goals):
    """A* nhieu diem voi h(n) la MST Manhattan cua cac diem con lai."""
    goal_to_bit, heuristic = make_mst_heuristic(goals)
    all_mask = (1 << len(goal_to_bit)) - 1
    start_mask = all_mask & ~goal_to_bit.get(start, 0)
    start_state = (start, start_mask)

    serial = count()
    start_h = heuristic(start, start_mask)
    open_heap = [(start_h, start_h, next(serial), 0, start_state)]
    open_active = {start_state}
    best_g = {start_state: 0}
    parent = {start_state: None}
    closed = set()
    expanded = 0
    open_max = 1

    while open_heap:
        f_value, h_value, _, g_value, state = heapq.heappop(open_heap)
        if state not in open_active or g_value != best_g.get(state):
            continue

        open_active.remove(state)
        if state in closed:
            continue

        closed.add(state)
        expanded += 1
        pos, remaining_mask = state

        if remaining_mask == 0:
            state_path = reconstruct_path(parent, state)
            position_path = [item[0] for item in state_path]
            return make_result(position_path, expanded, open_max)

        for nxt in valid_neighbors(grid, pos):
            new_mask = remaining_mask & ~goal_to_bit.get(nxt, 0)
            next_state = (nxt, new_mask)
            new_g = best_g[state] + 1

            if new_g >= best_g.get(next_state, float("inf")):
                continue

            best_g[next_state] = new_g
            parent[next_state] = state
            if next_state in closed:
                closed.remove(next_state)

            new_h = heuristic(nxt, new_mask)
            new_f = new_g + new_h
            open_active.add(next_state)
            heapq.heappush(
                open_heap,
                (new_f, new_h, next(serial), new_g, next_state),
            )
            open_max = max(open_max, len(open_active))

    return make_result(None, expanded, open_max)


def expand_grid(grid, scale=3):
    """Phong to moi o thanh khoi scale x scale de tao luoi 30x30."""
    expanded_grid = []
    for row in grid:
        for _ in range(scale):
            new_row = []
            for cell in row:
                value = 1 if cell == 1 else 0
                new_row.extend([value] * scale)
            expanded_grid.append(new_row)
    return expanded_grid


def scale_position(pos, scale=3):
    """Dua toa do vao tam khoi scale x scale tuong ung."""
    return (pos[0] * scale + scale // 2, pos[1] * scale + scale // 2)


def validate_path(grid, path, start, goal=None, required_goals=None):
    """Kiem tra duong di co hop le, khong dam vat can va di qua du diem hay khong."""
    if not path or path[0] != start:
        return False
    if goal is not None and path[-1] != goal:
        return False

    rows, cols = len(grid), len(grid[0])
    for r, c in path:
        if not (0 <= r < rows and 0 <= c < cols):
            return False
        if grid[r][c] == 1:
            return False

    for a, b in zip(path, path[1:]):
        if manhattan(a, b) != 1:
            return False

    if required_goals is not None and not set(required_goals).issubset(set(path)):
        return False
    return True


def average_runtime_ms(search_function, repeats=REPEATS):
    """Do thoi gian trung binh; cac chi so duong di lay tu mot lan chay rieng."""
    total_ns = 0
    for _ in range(repeats):
        start_ns = time.perf_counter_ns()
        search_function()
        total_ns += time.perf_counter_ns() - start_ns
    return total_ns / repeats / 1_000_000


def print_table(title, results):
    print("\n" + "=" * 94)
    print(title)
    print("=" * 94)
    print(f"{'Diem dich':<19} | {'Thuat toan':<10} | {'Buoc':>5} | {'Mo rong':>8} | {'OPEN Max':>8} | {'Thoi gian TB (ms)':>18}")
    print("-" * 94)
    for row in results:
        steps = str(row["So_buoc"]) if row["So_buoc"] is not None else "Khong co"
        print(
            f"{row['Diem_dich']:<19} | {row['Thuat_toan']:<10} | "
            f"{steps:>5} | {row['Node_mo_rong']:>8} | {row['OPEN_Max']:>8} | "
            f"{row['Thoi_gian_trung_binh_ms']:>18}"
        )


def run_single_goal_experiments():
    output_rows = []
    for target_name, goal in GOALS.items():
        algorithms = [
            ("BFS", lambda g=goal: bfs_search(CAMPUS_MAP, START, g)),
            ("GBFS", lambda g=goal: informed_search(CAMPUS_MAP, START, g, "GBFS")),
            ("A*", lambda g=goal: informed_search(CAMPUS_MAP, START, g, "A*")),
        ]

        for algorithm, search_fn in algorithms:
            result = search_fn()
            if not validate_path(CAMPUS_MAP, result["path"], START, goal=goal):
                raise RuntimeError(f"Duong di {algorithm} den {target_name} khong hop le")

            avg_ms = average_runtime_ms(search_fn)
            row = {
                "Diem_dich": target_name,
                "Thuat_toan": algorithm,
                "So_buoc": result["steps"],
                "Node_mo_rong": result["expanded"],
                "OPEN_Max": result["open_max"],
                "Thoi_gian_trung_binh_ms": f"{avg_ms:.6f}",
            }
            output_rows.append(row)

    print_table("THI NGHIEM 1 - MOT DIEM TREN LUOI 10x10", output_rows)

    # Kiem tra A* va BFS co cung chi phi toi uu tren tung diem hay khong.
    for target_name, goal in GOALS.items():
        bfs_result = bfs_search(CAMPUS_MAP, START, goal)
        astar_result = informed_search(CAMPUS_MAP, START, goal, "A*")
        assert bfs_result["steps"] == astar_result["steps"], (
            f"BFS va A* khong cung so buoc tai {target_name}"
        )
    return output_rows


def run_multi_goal_experiments():
    algorithms = [
        ("BFS ket hop", lambda: bfs_multi_goal(CAMPUS_MAP, START, GOALS)),
        ("GBFS gan nhat", lambda: gbfs_nearest_multi_goal(CAMPUS_MAP, START, GOALS)),
        ("A* MST", lambda: astar_mst_multi_goal(CAMPUS_MAP, START, GOALS)),
    ]
    output_rows = []
    required_goals = list(GOALS.values())

    for algorithm, search_fn in algorithms:
        result = search_fn()
        if not validate_path(
            CAMPUS_MAP,
            result["path"],
            START,
            required_goals=required_goals,
        ):
            raise RuntimeError(f"Duong di nhieu diem cua {algorithm} khong hop le")

        avg_ms = average_runtime_ms(search_fn)
        row = {
            "Diem_dich": "Ca 4 diem",
            "Thuat_toan": algorithm,
            "So_buoc": result["steps"],
            "Node_mo_rong": result["expanded"],
            "OPEN_Max": result["open_max"],
            "Thoi_gian_trung_binh_ms": f"{avg_ms:.6f}",
        }
        output_rows.append(row)

    print_table("THI NGHIEM 2 - HANH TRINH GIAO CA 4 DIEM TREN LUOI 10x10", output_rows)

    bfs_result = bfs_multi_goal(CAMPUS_MAP, START, GOALS)
    astar_result = astar_mst_multi_goal(CAMPUS_MAP, START, GOALS)
    assert bfs_result["steps"] == astar_result["steps"], (
        "BFS ket hop va A* MST khong cung chi phi hanh trinh"
    )
    return output_rows


def run_30x30_experiment():
    # Ban do 30x30 duoc tao bang cach mo rong moi o 10x10 thanh khoi 3x3.
    grid_30 = expand_grid(CAMPUS_MAP, GRID_SCALE)
    start_30 = scale_position(START, GRID_SCALE)
    goal_name = "C4 (D2-405)"
    goal_30 = scale_position(GOALS[goal_name], GRID_SCALE)

    assert len(grid_30) == 30 and len(grid_30[0]) == 30

    algorithms = [
        ("BFS", lambda: bfs_search(grid_30, start_30, goal_30)),
        ("GBFS", lambda: informed_search(grid_30, start_30, goal_30, "GBFS")),
        ("A*", lambda: informed_search(grid_30, start_30, goal_30, "A*")),
    ]
    output_rows = []

    for algorithm, search_fn in algorithms:
        result = search_fn()
        if not validate_path(grid_30, result["path"], start_30, goal=goal_30):
            raise RuntimeError(f"Duong di {algorithm} tren luoi 30x30 khong hop le")

        avg_ms = average_runtime_ms(search_fn)
        row = {
            "Diem_dich": "C4 tren luoi 30x30",
            "Thuat_toan": algorithm,
            "So_buoc": result["steps"],
            "Node_mo_rong": result["expanded"],
            "OPEN_Max": result["open_max"],
            "Thoi_gian_trung_binh_ms": f"{avg_ms:.6f}",
        }
        output_rows.append(row)

    print("\nBan do 30x30: mo rong moi o cua ma tran 10x10 thanh khoi 3x3.")
    print(f"Diem bat dau: {start_30}; diem dich: {goal_30}; kich thuoc: {len(grid_30)}x{len(grid_30[0])}.")
    print_table("THI NGHIEM 3 - TIM DUONG TREN LUOI 30x30", output_rows)

    bfs_result = bfs_search(grid_30, start_30, goal_30)
    astar_result = informed_search(grid_30, start_30, goal_30, "A*")
    assert bfs_result["steps"] == astar_result["steps"], (
        "BFS va A* khong cung so buoc tren luoi 30x30"
    )
    return output_rows


def main():
    print("ROBOT GIAO HANG NEU - CHUONG TRINH THUC NGHIEM")
    print(f"Moi thuat toan duoc lap {REPEATS} lan de tinh thoi gian trung binh.")
    print("Expanded Nodes tinh ca node dich khi node dich duoc lay ra khoi OPEN.")
    print("OPEN Max la so trang thai dang cho lon nhat trong OPEN.")

    run_single_goal_experiments()
    run_multi_goal_experiments()
    run_30x30_experiment()

    print("\nDa hoan thanh tat ca thi nghiem.")


if __name__ == "__main__":
    main()