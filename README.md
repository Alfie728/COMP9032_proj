# COMP9032 Project — Feature Checklist & Flow

## 1. Input Phase
- [x] Input accident scene & visibility distance  
  - Display input values on LCD
- Push buttons PB0/PB1 live on the lab board's PORTD header (RDX4/RDX3). No wiring is required on PORTB.

---

## 2. Generate Search Path (PB0)
- [ ] **Press PB0** to generate search path
- [ ] **Search Path Display on LCD**
  - First line: full generated search path  
  - Sequence scrolls **right → left**

---

## 3. Start Search (PB1)
- [ ] **PB1: Begin search simulation**

### Display During Search
- [ ] **First line:**  
  - A segment of the search path, beginning with the **current observation point**  
  - Updated when drone flies to the next observation point  

- [ ] **Second line:**  
  - State  
  - Altitude  
  - Speed  

---

## 4. Drone Control (Keypad)
- [ ] Control **speed** and **altitude** of drone

### Key Assignments
- [ ] **UA** — Altitude Up  
- [ ] **DA** — Altitude Down  
- [ ] **US** — Speed Up  
- [ ] **DS** — Speed Down  

---

## 5. After Searching
- [ ] Display result:
  - “Not Found” **or**  
  - Accident location (x, y)

---

## 6. LED Bar Behavior
- [ ] LED bar is **on** when simulation starts  
- [ ] LED bar **flashes when drone crashes**

