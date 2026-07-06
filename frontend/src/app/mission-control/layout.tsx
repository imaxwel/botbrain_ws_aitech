'use client';

import RobotHeader from '@/components/robot-header';
import SeparatorLine from '@/components/ui/separator-line';
import { ExtrasBar } from '@/components/extras-bar';
import ProtectedRoute from '@/components/protected-route';
import { MissionSupervisorProvider } from '@/contexts/MissionSupervisorContext';

export default function MissionControlLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  const content = (
    <MissionSupervisorProvider>
      <div
        className="min-w-screen min-h-screen bg-clear-pink dark:bg-botbot-darkest text-black dark:text-white relative overflow-hidden"
        style={{ isolation: 'isolate' }}
      >
        <RobotHeader />
        <SeparatorLine />
        <ExtrasBar />
        <main className="p-0 relative z-0 h-[calc(100vh-70px-24px)] overflow-hidden">
          {children}
        </main>
      </div>
    </MissionSupervisorProvider>
  );

  if (process.env.NEXT_PUBLIC_REQUIRE_AUTH_FOR_MISSION_CONTROL === 'true') {
    return <ProtectedRoute>{content}</ProtectedRoute>;
  }

  return content;
}
