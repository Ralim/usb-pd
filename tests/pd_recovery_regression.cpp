// Each case runs in a fresh process so the engine starts with zeroed static state.

#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>
// Keep state injection in the test harness, without changing the production API.
#define private public
#include "policy_engine.h"
#undef private
#include "fusb302_defines.h"
static std::deque<uint8_t> rx;
static std::vector<uint8_t> tx;
static uint8_t irqA;
bool read(uint8_t, uint8_t reg, uint8_t size, uint8_t *buf) {
  std::memset(buf, 0, size);
  if (reg == FUSB_FIFOS) {
    assert(rx.size() >= size);
    for (unsigned i = 0; i < size; ++i) { buf[i] = rx.front(); rx.pop_front(); }
  } else if (reg == FUSB_STATUS1) {
    buf[0] = rx.empty() ? FUSB_STATUS1_RX_EMPTY : 0;
  } else if (reg == FUSB_STATUS0A) {
    assert(size == 7); buf[2] = irqA;
  }
  return true;
}
bool write(uint8_t, uint8_t reg, uint8_t size, uint8_t *buf) {
  if (reg == FUSB_FIFOS) tx.insert(tx.end(), buf, buf + size);
  return true;
}
void delay(uint32_t) {}
uint32_t timestamp() { return 0; }
void sink(pd_msg *, bool) {}
bool eval(const pd_msg *, pd_msg *) { return false; }
bool epr(const epr_pd_msg *, pd_msg *) { return false; }
static FUSB302 f(0x44, read, write, delay);
static PolicyEngine engine(f, timestamp, delay, sink, eval, epr, 0);
void queue(uint16_t header) {
  rx.push_back(FUSB_FIFO_RX_SOP);
  rx.push_back(header & 0xff); rx.push_back(header >> 8);
  for (int i = 0; i < 4; ++i) rx.push_back(0);
}

uint16_t transmittedHeader() {
  assert(tx.size() >= 7);
  return uint16_t(tx[5]) | (uint16_t(tx[6]) << 8);
}
pd_msg control(uint8_t type, unsigned id = 0) {
  pd_msg msg{};
  msg.hdr = PD_SPECREV_3_0 | type | (id << PD_HDR_MESSAGEID_SHIFT);
  return msg;
}
void receivedReset(bool ready, uint16_t revision = PD_SPECREV_3_0) {
  engine.hdr_template = revision;
  engine._tx_messageidcounter = 5;
  engine._explicit_contract = true;
  engine.state = ready ? PolicyEngine::PESinkReady : PolicyEngine::PEWaitingEvent;
  engine.waitingEventsTimeout = 0xffffffff;
  engine.postNotificationEvalState = PolicyEngine::PESinkReady;
  auto stale = control(PD_MSGTYPE_GET_SOURCE_CAP_EXTENDED, 7);
  engine.incomingMessages.push(&stale);
  if (!ready) engine.currentEvents = uint32_t(PolicyEngine::Notifications::TIMEOUT);
  queue(revision | PD_POWERROLE_SOURCE | PD_MSGTYPE_SOFT_RESET);
  engine.readPendingMessage();
  engine.thread();
  pd_msg sent{}; sent.hdr = transmittedHeader();
  assert(PD_MSGTYPE_GET(&sent) == PD_MSGTYPE_ACCEPT);
  assert(PD_NUMOBJ_GET(&sent) == 0 && PD_MESSAGEID_GET(&sent) == 0);
  assert((sent.hdr & PD_HDR_SPECREV) == revision);
  assert(engine.incomingMessages.getOccupied() == 0);
  assert(engine.hasExplicitContract()); // no forced power reset
  assert(engine.postSendState == PolicyEngine::PESinkSetupWaitCap);
  auto crc = control(PD_MSGTYPE_GOODCRC);
  crc.hdr = revision | PD_MSGTYPE_GOODCRC;
  engine.incomingMessages.push(&crc);
  assert(engine.pe_sink_wait_good_crc() == PolicyEngine::PESinkSetupWaitCap);
  assert(engine._tx_messageidcounter == 1);
}
void sentReset() {
  engine.hdr_template = PD_SPECREV_3_0;
  engine._tx_messageidcounter = 6;
  auto stale = control(PD_MSGTYPE_GET_SINK_CAP);
  engine.incomingMessages.push(&stale);
  engine.pe_sink_send_soft_reset();
  pd_msg sent{}; sent.hdr = transmittedHeader();
  assert(PD_MSGTYPE_GET(&sent) == PD_MSGTYPE_SOFT_RESET);
  assert(PD_MESSAGEID_GET(&sent) == 0 && PD_NUMOBJ_GET(&sent) == 0);
  assert(engine.incomingMessages.getOccupied() == 0);
  assert(engine.postSendState == PolicyEngine::PESinkSendSoftResetTxOK);
}
void interleavedMessages(bool matchingCRC) {
  engine.hdr_template = PD_SPECREV_3_0;
  engine._tx_messageidcounter = 2;
  engine.postSendState = PolicyEngine::PESinkReady;
  engine.postSendFailedState = PolicyEngine::PESinkSendSoftReset;
  auto first = control(PD_MSGTYPE_GET_SOURCE_CAP_EXTENDED, 7);
  auto crc = control(PD_MSGTYPE_GOODCRC, matchingCRC ? 2 : 1);
  auto second = control(PD_MSGTYPE_GET_SINK_CAP, 3);
  engine.incomingMessages.push(&first);
  engine.incomingMessages.push(&crc);
  engine.incomingMessages.push(&second);
  auto next = engine.pe_sink_wait_good_crc();
  assert(next == (matchingCRC ? PolicyEngine::PESinkReady : PolicyEngine::PESinkSendSoftReset));
  assert(engine._tx_messageidcounter == (matchingCRC ? 3 : 2));
  assert(engine.incomingMessages.getOccupied() == 2);
  if (matchingCRC) {
    // The preserved Get_Source_Cap_Extended must receive Not_Supported.
    engine.pe_sink_ready(); // schedules queued MSG_RX after TX_DONE
    assert(engine.pe_sink_ready() == PolicyEngine::PESinkSendNotSupported);
    engine.pe_sink_send_not_supported();
    pd_msg sent{}; sent.hdr = transmittedHeader();
    assert(PD_MSGTYPE_GET(&sent) == PD_MSGTYPE_NOT_SUPPORTED && PD_NUMOBJ_GET(&sent) == 0);
    assert(PD_MESSAGEID_GET(&sent) == 3);
  } else {
    pd_msg kept{}; engine.incomingMessages.pop(&kept); assert(kept.hdr == first.hdr);
  }
  pd_msg kept{}; engine.incomingMessages.pop(&kept); assert(kept.hdr == second.hdr);
}
void ordinaryNegotiation() {
  for (auto rev : {PD_SPECREV_2_0, PD_SPECREV_3_0}) {
    engine.currentEvents = 0;
    engine.incomingMessages.flush();
    engine.hdr_template = rev;
    engine._explicit_contract = false;
    engine._last_dpm_request.hdr = rev | PD_MSGTYPE_REQUEST | PD_NUMOBJ(1);
    auto accept = control(PD_MSGTYPE_ACCEPT);
    engine.incomingMessages.push(&accept);
    assert(engine.pe_sink_wait_cap_resp() == PolicyEngine::PEWaitingEvent);
    assert(engine.postNotificationEvalState == PolicyEngine::PESinkTransitionSink);
    auto ready = control(PD_MSGTYPE_PS_RDY);
    engine.incomingMessages.push(&ready);
    engine.state = PolicyEngine::PEWaitingEvent;
    engine.notify(PolicyEngine::Notifications::MSG_RX);
    engine.thread(); engine.thread();
    assert(engine.state == PolicyEngine::PESinkReady && engine.hasExplicitContract());
    assert((engine.hdr_template & PD_HDR_SPECREV) == rev);
  }
}
void fullQueue() {
  for (unsigned offset = 0; offset < 4; ++offset) {
    for (unsigned crcPosition = 0; crcPosition < 4; ++crcPosition) {
      engine.incomingMessages.flush();
      auto dummy = control(PD_MSGTYPE_PING);
      for (unsigned i = 0; i < offset; ++i) {
        engine.incomingMessages.push(&dummy); engine.incomingMessages.pop(nullptr);
      }
      engine._tx_messageidcounter = 2;
      engine.postSendState = PolicyEngine::PESinkReady;
      uint16_t expected[3]; unsigned index = 0;
      for (unsigned i = 0; i < 4; ++i) {
        auto message = control(i == crcPosition ? PD_MSGTYPE_GOODCRC : PD_MSGTYPE_GET_SOURCE_CAP_EXTENDED,
                               i == crcPosition ? 2 : index);
        if (i != crcPosition) expected[index++] = message.hdr;
        engine.incomingMessages.push(&message);
      }
      assert(engine.incomingMessages.getOccupied() == 4);
      assert(engine.pe_sink_wait_good_crc() == PolicyEngine::PESinkReady);
      assert(engine.incomingMessages.getOccupied() == 3);
      for (auto header : expected) {
        pd_msg message{}; engine.incomingMessages.pop(&message); assert(message.hdr == header);
      }
    }
  }
}
void duplicateAcknowledgements() {
  engine._tx_messageidcounter = 2;
  engine.postSendState = PolicyEngine::PESinkReady;
  auto old = control(PD_MSGTYPE_GOODCRC, 1);
  auto query = control(PD_MSGTYPE_GET_SINK_CAP, 4);
  auto crc = control(PD_MSGTYPE_GOODCRC, 2);
  engine.incomingMessages.push(&old); engine.incomingMessages.push(&query);
  engine.incomingMessages.push(&crc); engine.incomingMessages.push(&crc);
  assert(engine.pe_sink_wait_good_crc() == PolicyEngine::PESinkReady);
  assert(engine._tx_messageidcounter == 3 && engine.incomingMessages.getOccupied() == 1);
  pd_msg kept{}; engine.incomingMessages.pop(&kept); assert(kept.hdr == query.hdr);
}
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  if (!strcmp(argv[1], "received_reset_timeout")) receivedReset(false);
  else if (!strcmp(argv[1], "outgoing_reset")) sentReset();
  else if (!strcmp(argv[1], "interleaved_requests")) interleavedMessages(true);
  else if (!strcmp(argv[1], "mismatched_ack")) interleavedMessages(false);
  else if (!strcmp(argv[1], "pd2_pd3_completion")) ordinaryNegotiation();
  else if (!strcmp(argv[1], "received_reset_ready")) receivedReset(true);
  else if (!strcmp(argv[1], "wrapped_full_queue")) fullQueue();
  else if (!strcmp(argv[1], "duplicate_ack")) duplicateAcknowledgements();
  else if (!strcmp(argv[1], "pd2_received_reset")) receivedReset(false, PD_SPECREV_2_0);
  else return 2;
  puts("PASS");
}
